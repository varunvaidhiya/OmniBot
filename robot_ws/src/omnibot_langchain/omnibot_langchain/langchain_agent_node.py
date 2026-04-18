#!/usr/bin/env python3
"""
LangChain AI orchestration node for OmniBot.

Subscribes to /ai/command (natural language String) and uses a Claude-powered
LangChain ReAct agent to decompose the request into structured robot commands,
publishing to /mission/command and /mission/cancel (consumed by mission_planner).

Runs on the AI desktop PC alongside the VLA nodes, NOT on the Raspberry Pi.

Usage:
  export ANTHROPIC_API_KEY=sk-ant-...
  ros2 launch omnibot_langchain langchain_agent.launch.py

  # Send a natural language command
  ros2 topic pub --once /ai/command std_msgs/msg/String \\
    "data: 'Fetch the coffee cup from the kitchen and bring it to me'"
"""

import concurrent.futures
import os
import threading
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

from omnibot_langchain.memory.entity_memory import EntityMemory
from omnibot_langchain.vision.scene_describer import SceneDescriber
from omnibot_langchain.tools.navigation_tools import make_navigation_tools
from omnibot_langchain.tools.vla_tools import make_vla_tools
from omnibot_langchain.tools.status_tools import make_status_tools
from omnibot_langchain.tools.query_tools import make_query_tools
from omnibot_langchain.tools.human_tools import make_human_tools
from omnibot_langchain.agent.prompt import build_system_prompt
from omnibot_langchain.agent.react_agent import build_agent


class LangchainAgentNode(Node):
    """LangChain ReAct agent as a ROS 2 node."""

    def __init__(self):
        super().__init__('langchain_agent_node')

        self._declare_parameters()

        p = self.get_parameter
        self._api_key: str = p('anthropic_api_key').value
        self._vla_serve_url: str = p('vla_serve_url').value
        self._locations_yaml: str = p('locations_yaml').value
        self._entity_memory_path: str = p('entity_memory_path').value
        self._max_iterations: int = p('agent_max_iterations').value
        self._timeout: float = p('agent_timeout_sec').value
        self._memory_window_k: int = p('memory_window_k').value
        self._use_claude_vision: bool = p('use_claude_vision').value

        # ── Thread-shared state ────────────────────────────────────────────
        self._image_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._latest_front_image: Optional[np.ndarray] = None
        self._latest_wrist_image: Optional[np.ndarray] = None
        self._latest_mission_status: Optional[str] = None

        # ── ROS interfaces ─────────────────────────────────────────────────
        self._setup_ros_interfaces()

        # ── Locations ─────────────────────────────────────────────────────
        self._locations: Dict[str, Any] = {}
        self._load_locations()

        # ── Memory ────────────────────────────────────────────────────────
        self._entity_memory = EntityMemory(self._entity_memory_path)

        # ── Vision ────────────────────────────────────────────────────────
        self._vision = SceneDescriber(
            node=self,
            use_claude_vision=self._use_claude_vision,
            vla_serve_url=self._vla_serve_url,
            anthropic_api_key=self._api_key,
        )

        # ── LangChain agent ───────────────────────────────────────────────
        self._tools: List = self._build_tools()
        self._chat_history: List = []
        self._history_lock = threading.Lock()
        self._agent: Optional[Any] = None
        self._build_langchain_agent()

        # ── Thread pool (max 1 concurrent agent call) ──────────────────────
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self._active_future: Optional[concurrent.futures.Future] = None

        self.get_logger().info('LangChain agent node ready. Listening on /ai/command.')

    # ── Parameter declarations ─────────────────────────────────────────────

    def _declare_parameters(self) -> None:
        self.declare_parameter('anthropic_api_key', '')
        self.declare_parameter('vla_serve_url', 'http://localhost:8000')
        self.declare_parameter('locations_yaml', '')
        self.declare_parameter('entity_memory_path', '~/.omnibot/entity_memory.json')
        self.declare_parameter('agent_max_iterations', 10)
        self.declare_parameter('agent_timeout_sec', 120.0)
        self.declare_parameter('memory_window_k', 10)
        self.declare_parameter('use_claude_vision', True)
        self.declare_parameter('use_sim_time', False)

    # ── ROS setup ──────────────────────────────────────────────────────────

    def _setup_ros_interfaces(self) -> None:
        qos = rclpy.qos.QoSProfile(depth=10)

        self._mission_pub = self.create_publisher(String, '/mission/command', qos)
        self._cancel_pub = self.create_publisher(String, '/mission/cancel', qos)
        self._status_pub = self.create_publisher(String, '/ai/status', qos)
        self._response_needed_pub = self.create_publisher(String, '/ai/response_needed', qos)

        self.create_subscription(String, '/ai/command', self._on_ai_command, qos)
        self.create_subscription(String, '/mission/status', self._on_mission_status, qos)

        if CV_BRIDGE_AVAILABLE:
            self._bridge = CvBridge()
            self.create_subscription(
                Image, '/camera/front/image_raw', self._on_front_image, qos)
            self.create_subscription(
                Image, '/camera/wrist/image_raw', self._on_wrist_image, qos)
        else:
            self._bridge = None
            self.get_logger().warn(
                'cv_bridge not available — scene description from camera disabled.')

    # ── Location loading ───────────────────────────────────────────────────

    def _load_locations(self) -> None:
        path = self._locations_yaml
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_dir = get_package_share_directory('omnibot_hybrid')
                path = os.path.join(pkg_dir, 'config', 'named_locations.yaml')
            except Exception:
                self.get_logger().warn(
                    'Could not resolve omnibot_hybrid share directory. '
                    'Set locations_yaml parameter explicitly.')
                return

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self._locations = data.get('locations', {}) or {}
            self.get_logger().info(
                f'Loaded {len(self._locations)} locations from {path}')
        except FileNotFoundError:
            self.get_logger().warn(f'Locations file not found: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load locations: {e}')

    # ── Agent construction ─────────────────────────────────────────────────

    def _build_tools(self) -> List:
        tools = []
        tools.extend(make_navigation_tools(self))
        tools.extend(make_vla_tools(self))
        tools.extend(make_status_tools(self))
        tools.extend(make_query_tools(self))
        tools.extend(make_human_tools(self))
        return tools

    def _build_langchain_agent(self) -> None:
        try:
            system_prompt = build_system_prompt(
                locations=list(self._locations.keys()),
                entity_memory_summary=self._entity_memory.get_summary(),
            )
            self._agent = build_agent(
                tools=self._tools,
                system_prompt=system_prompt,
                api_key=self._api_key,
                max_iterations=self._max_iterations,
                timeout=self._timeout,
            )
            self.get_logger().info('LangChain agent built successfully.')
        except ImportError as e:
            self.get_logger().fatal(
                f'Failed to build agent — missing package: {e}. '
                'Run: pip install -r requirements.txt in omnibot_langchain/'
            )
            self._agent = None
        except Exception as e:
            self.get_logger().fatal(f'Failed to build agent: {e}')
            self._agent = None

    # ── ROS callbacks ──────────────────────────────────────────────────────

    def _on_ai_command(self, msg: String) -> None:
        if self._agent is None:
            self._publish_ai_status(
                'ERROR: Agent not initialized. Check ANTHROPIC_API_KEY and dependencies.')
            return

        if self._active_future and not self._active_future.done():
            self.get_logger().warn(
                'Agent is busy processing a previous command — ignoring new command. '
                f'Ignored: "{msg.data}"'
            )
            self._publish_ai_status('BUSY: Agent is already processing a command.')
            return

        self.get_logger().info(f'[AI] Received command: "{msg.data}"')
        self._publish_ai_status(f'PROCESSING: {msg.data}')
        self._active_future = self._executor.submit(self._run_agent, msg.data)

    def _on_mission_status(self, msg: String) -> None:
        with self._status_lock:
            self._latest_mission_status = msg.data

    def _on_front_image(self, msg: 'Image') -> None:
        if self._bridge is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            with self._image_lock:
                self._latest_front_image = np.array(cv_img)
        except Exception as e:
            self.get_logger().warn(
                f'Front image conversion error: {e}', throttle_duration_sec=5.0)

    def _on_wrist_image(self, msg: 'Image') -> None:
        if self._bridge is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            with self._image_lock:
                self._latest_wrist_image = np.array(cv_img)
        except Exception as e:
            self.get_logger().warn(
                f'Wrist image conversion error: {e}', throttle_duration_sec=5.0)

    # ── Agent execution (runs in thread pool) ──────────────────────────────

    def _run_agent(self, text: str) -> None:
        try:
            with self._history_lock:
                history = list(self._chat_history)

            result = self._agent.invoke({
                'input': text,
                'chat_history': history,
            })

            output = result.get('output', str(result))

            # Update sliding-window conversation history
            with self._history_lock:
                from langchain_core.messages import HumanMessage, AIMessage
                self._chat_history.append(HumanMessage(content=text))
                self._chat_history.append(AIMessage(content=output))
                # Keep last k*2 messages (k human + k AI)
                max_msgs = self._memory_window_k * 2
                if len(self._chat_history) > max_msgs:
                    self._chat_history = self._chat_history[-max_msgs:]

            self.get_logger().info(f'[AI] Agent output: {output}')
            self._publish_ai_status(f'DONE: {output}')

        except TimeoutError:
            msg = 'Agent timed out. The task may be too complex or the API is slow.'
            self.get_logger().warn(msg)
            self._publish_ai_status(f'TIMEOUT: {msg}')
        except Exception as e:
            msg = f'Agent error: {e}'
            self.get_logger().error(msg)
            self._publish_ai_status(f'ERROR: {msg}')

    # ── Publisher helpers (called from tools via self) ─────────────────────

    def _publish_mission(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._mission_pub.publish(msg)
        self.get_logger().info(f'[Mission] Published: "{command}"')

    def _publish_cancel(self, reason: str = 'cancel') -> None:
        msg = String()
        msg.data = reason
        self._cancel_pub.publish(msg)
        self.get_logger().info('[Mission] Cancel published.')

    def _publish_ai_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._executor.shutdown(wait=False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LangchainAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
