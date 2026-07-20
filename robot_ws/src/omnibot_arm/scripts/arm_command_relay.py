#!/usr/bin/env python3
"""
arm_command_relay.py — direct arm-command bridge for OmniBot Open.

`arm_driver_node` subscribes to ``/arm/joint_commands/out``, which in the full
build is produced by ``arm_cmd_mux`` (in the closed ``omnibot_rl`` package) as it
arbitrates between SmolVLA and RL command sources. That mux is **not** part of
OmniBot Open, so without a bridge the manual command topic documented for the
Android app and teleop — ``/arm/joint_commands`` — never reaches the driver.

This node is that bridge. It simply republishes ``/arm/joint_commands`` →
``/arm/joint_commands/out`` so the open build works end-to-end. When you add the
full command mux, launch with ``direct_command:=false`` to disable this relay and
let the mux own the ``/out`` topic.

Topics subscribed:
  /arm/joint_commands      (sensor_msgs/JointState) - manual / app commands
Topics published:
  /arm/joint_commands/out  (sensor_msgs/JointState) - what the driver consumes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmCommandRelay(Node):
    """Passthrough from the manual command topic to the driver's input."""

    def __init__(self):
        super().__init__("arm_command_relay")
        self.declare_parameter("input_topic", "/arm/joint_commands")
        self.declare_parameter("output_topic", "/arm/joint_commands/out")

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(JointState, out_topic, 10)
        self.create_subscription(JointState, in_topic, self._relay_cb, 10)

        self.get_logger().info(
            f"ArmCommandRelay: {in_topic} → {out_topic} "
            "(disable with direct_command:=false when using the full command mux)"
        )

    def _relay_cb(self, msg: JointState):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
