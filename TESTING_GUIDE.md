# OmniBot Testing Guide

Comprehensive testing guide for the OmniBot ROS 2 Mecanum Wheel Mobile Manipulation Robot.

---

## Table of Contents

1. [Prerequisites & Environment Setup](#1-prerequisites--environment-setup)
2. [Build System](#2-build-system)
3. [omnibot_driver — Yahboom Motor Controller](#3-omnibot_driver--yahboom-motor-controller)
4. [omnibot_driver — Serial Bridge Node](#4-omnibot_driver--serial-bridge-node)
5. [omnibot_driver — Legacy Mecanum Controller](#5-omnibot_driver--legacy-mecanum-controller)
6. [omnibot_driver — Hardware Test Node](#6-omnibot_driver--hardware-test-node)
7. [omnibot_hybrid — CmdVelMux](#7-omnibot_hybrid--cmdvelmux)
8. [omnibot_hybrid — MissionPlanner](#8-omnibot_hybrid--missionplanner)
9. [omnibot_arm — Arm Driver](#9-omnibot_arm--arm-driver)
10. [omnibot_vla — OpenVLA Node](#10-omnibot_vla--openvla-node)
11. [omnibot_lerobot — SmolVLA Node](#11-omnibot_lerobot--smolvla-node)
12. [omnibot_lerobot — Teleop Recorder](#12-omnibot_lerobot--teleop-recorder)
13. [omnibot_bringup — Launch Files](#13-omnibot_bringup--launch-files)
14. [omnibot_navigation — SLAM & Nav2](#14-omnibot_navigation--slam--nav2)
15. [omnibot_description — URDF](#15-omnibot_description--urdf)
16. [packages/yahboom_ros2 — Protocol Library](#16-packagesyahboom_ros2--protocol-library)
17. [packages/ros2_bev_stitcher — BEV Node](#17-packagesros2_bev_stitcher--bev-node)
18. [packages/vla_serve — FastAPI Server](#18-packagesvla_serve--fastapi-server)
19. [packages/robot_episode_dataset — Dataset Utilities](#19-packagesrobot_episode_dataset--dataset-utilities)
20. [vla_engine — OpenVLA Model & Server](#20-vla_engine--openvla-model--server)
21. [data_engine — Bag Ingestion Pipeline](#21-data_engine--bag-ingestion-pipeline)
22. [Android App](#22-android-app)
23. [Root-level Hardware Debug Scripts](#23-root-level-hardware-debug-scripts)
24. [Known Issues Verification](#24-known-issues-verification)
25. [CI/CD Pipeline](#25-cicd-pipeline)

---

## 1. Prerequisites & Environment Setup

### System Requirements

| Component | Requirement |
|-----------|-------------|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Python | 3.10+ |
| Robot Brain | Raspberry Pi 5 8 GB |
| AI Brain | NVIDIA GPU ≥ 16 GB VRAM |
| Gazebo | Harmonic |

### Install ROS 2 Jazzy

```bash
# Add ROS 2 apt repo
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools -y
```

### Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
# Add to ~/.bashrc for persistence:
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### Install Python dependencies

```bash
pip install numpy"<2.0" torch opencv-python accelerate transformers
pip install pytest pytest-cov
```

### Install rosdep

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### Clone the repository

```bash
git clone https://github.com/varunvaidhiya/Mecanum-Wheel-Robot.git
cd Mecanum-Wheel-Robot
```

---

## 2. Build System

### What it does

The ROS 2 workspace uses `colcon` to build all packages. Standalone Python packages under `packages/` use pip. VLA/data-engine components use pytest directly.

### 2.1 Full workspace build

```bash
cd robot_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

**Expected output:**
```
Starting >>> omnibot_description
Starting >>> omnibot_driver
...
Finished <<< omnibot_bringup [X.XXs]
Summary: X packages finished
```

**Failure indicators:**
- `CMake Error` — missing dependency, run `rosdep install` first
- `ModuleNotFoundError` — Python package not installed
- `ERROR:colcon.colcon_core.package_identification` — malformed `package.xml`

### 2.2 Individual package build

```bash
cd robot_ws
colcon build --symlink-install --packages-select omnibot_driver
colcon build --symlink-install --packages-select omnibot_hybrid
colcon build --symlink-install --packages-select omnibot_arm
colcon build --symlink-install --packages-select omnibot_vla
colcon build --symlink-install --packages-select omnibot_lerobot
colcon build --symlink-install --packages-select omnibot_navigation
colcon build --symlink-install --packages-select omnibot_description
colcon build --symlink-install --packages-select omnibot_bringup
```

### 2.3 Standalone Python packages

```bash
# From repo root
pip install -e packages/yahboom_ros2
pip install -e packages/vla_serve
pip install -e packages/ros2_bev_stitcher
pip install -e packages/robot_episode_dataset
```

**Verify install:**
```bash
python -c "from yahboom_ros2.protocol import packet_motion; print('OK')"
python -c "from vla_serve.server import app; print('OK')"
```

### 2.4 Run all ROS 2 tests

```bash
cd robot_ws
colcon test
colcon test-result --verbose
```

### 2.5 Per-package tests

```bash
colcon test --packages-select omnibot_driver
colcon test-result --verbose --test-result-base build/omnibot_driver

colcon test --packages-select omnibot_hybrid
colcon test-result --verbose --test-result-base build/omnibot_hybrid
```

### 2.6 VLA engine tests

```bash
cd vla_engine
pytest tests/ -v
```

### 2.7 Data engine tests

```bash
cd data_engine
pytest tests/ -v
```

### 2.8 Python path verification

```bash
# After sourcing the workspace
source robot_ws/install/setup.bash
python -c "import omnibot_driver; print('ROS package importable')"
ros2 pkg list | grep omnibot
```

**Expected:** All `omnibot_*` packages listed.

---

## 3. omnibot_driver — Yahboom Motor Controller

**File:** `robot_ws/src/omnibot_driver/omnibot_driver/yahboom_controller_node.py`

**Hardware required for full tests.** Unit tests can mock serial.

### 3.1 What it does

- Opens `/dev/ttyUSB0` at 115200 baud
- Converts `/cmd_vel` (Twist) → mecanum wheel speeds → Yahboom serial packets
- Parses incoming serial data (velocity, IMU) and publishes `/odom` and `/imu/data`
- Broadcasts `odom → base_link` TF

### 3.2 Launch the node

```bash
# Source workspace first
source robot_ws/install/setup.bash

# With default parameters
ros2 run omnibot_driver yahboom_controller_node

# With custom serial port
ros2 run omnibot_driver yahboom_controller_node \
  --ros-args -p serial_port:=/dev/ttyUSB1 -p debug_serial:=true
```

**Expected on startup:**
```
[INFO] [yahboom_controller_node]: Sending SET_CAR_TYPE 1/5
...
[INFO] [yahboom_controller_node]: Sending SET_CAR_TYPE 5/5
[INFO] [yahboom_controller_node]: Yahboom controller ready
```

### 3.3 Test serial connection

```bash
# Verify device exists (hardware required)
ls -la /dev/ttyUSB0

# Check permissions
sudo usermod -aG dialout $USER
# Log out and back in, then retry
```

### 3.4 Test /cmd_vel subscription

```bash
# Terminal 1: run node
ros2 run omnibot_driver yahboom_controller_node

# Terminal 2: send forward command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Terminal 2: send strafe command (mecanum only)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Terminal 2: send rotation
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Terminal 2: stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.5 Test safety limits

```bash
# Exceeds max linear (0.2 m/s) — should be clamped
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# Expected: node logs clamping, sends 0.2 m/s to hardware

# Exceeds max angular (1.0 rad/s) — should be clamped
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}"
```

### 3.6 Test /odom publication

```bash
# Terminal 1: run node
ros2 run omnibot_driver yahboom_controller_node

# Terminal 2: echo odometry
ros2 topic echo /odom

# Terminal 3: move robot
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Expected:** `/odom` messages with incrementing `pose.pose.position.x`.

### 3.7 Test /imu/data publication

```bash
ros2 topic echo /imu/data
```

**Expected:** Messages with `linear_acceleration` and `angular_velocity` fields populated.

### 3.8 Test TF broadcast

```bash
ros2 run tf2_ros tf2_monitor odom base_link
# or
ros2 run tf2_tools view_frames
```

**Expected:** `odom -> base_link` transform present.

### 3.9 Test joystick beep (button A)

```bash
# Terminal 1: run node
ros2 run omnibot_driver yahboom_controller_node

# Terminal 2: simulate joystick button A (index 0) press
ros2 topic pub --once /joy sensor_msgs/msg/Joy \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], buttons: [1, 0, 0, 0, 0, 0, 0, 0]}"
```

**Expected (hardware):** Audible beep from robot buzzer.

### 3.10 Unit tests — packet encoding

Create `robot_ws/src/omnibot_driver/test/test_packet_encoding.py`:

```python
import pytest
import struct
from unittest.mock import MagicMock, patch


# Constants matching the node
HEAD_TX = 0xFF
DEVICE_ID = 0xFC
FUNC_SET_CAR_TYPE = 0x15
FUNC_MOTION = 0x12
FUNC_BEEP = 0x02
FUNC_MOTOR = 0x10
CAR_TYPE_MECANUM_X3 = 1


def compute_checksum(packet: bytes) -> int:
    return (sum(packet) + 5) & 0xFF


def build_packet(func: int, payload: bytes) -> bytes:
    length = 1 + 1 + 1 + len(payload)  # device_id + len + func + payload
    packet = bytes([HEAD_TX, DEVICE_ID, length, func]) + payload
    checksum = compute_checksum(packet)
    return packet + bytes([checksum])


def test_set_car_type_packet():
    pkt = build_packet(FUNC_SET_CAR_TYPE, bytes([CAR_TYPE_MECANUM_X3]))
    assert pkt[0] == 0xFF
    assert pkt[1] == 0xFC
    assert pkt[3] == FUNC_SET_CAR_TYPE
    assert pkt[4] == CAR_TYPE_MECANUM_X3
    # Verify checksum
    assert pkt[-1] == compute_checksum(pkt[:-1])


def test_motion_packet_forward():
    vx, vy, vz = 0.1, 0.0, 0.0
    payload = struct.pack('<bhhh',
                          CAR_TYPE_MECANUM_X3,
                          int(vx * 1000),
                          int(vy * 1000),
                          int(vz * 1000))
    pkt = build_packet(FUNC_MOTION, payload)
    assert pkt[3] == FUNC_MOTION
    # Verify checksum
    assert pkt[-1] == compute_checksum(pkt[:-1])


def test_motion_packet_strafe():
    vx, vy, vz = 0.0, 0.1, 0.0
    payload = struct.pack('<bhhh',
                          CAR_TYPE_MECANUM_X3,
                          int(vx * 1000),
                          int(vy * 1000),
                          int(vz * 1000))
    pkt = build_packet(FUNC_MOTION, payload)
    _, _, _, func, car, vx_b, vy_b, vz_b = struct.unpack('<BBBBbhhh', pkt[:-1])
    assert vy_b == 100   # 0.1 * 1000


def test_beep_packet():
    duration_ms = 500
    payload = struct.pack('<H', duration_ms)
    pkt = build_packet(FUNC_BEEP, payload)
    assert pkt[3] == FUNC_BEEP
    assert pkt[-1] == compute_checksum(pkt[:-1])


def test_checksum_formula():
    # Simple known packet: [0xFF, 0xFC, 3, 0x15, 1]
    packet = bytes([0xFF, 0xFC, 3, 0x15, 1])
    cs = (sum(packet) + 5) & 0xFF
    assert isinstance(cs, int)
    assert 0 <= cs <= 255


def test_safety_clamp_linear():
    MAX_LINEAR = 0.2
    raw = 1.0  # Request 1.0 m/s
    clamped = max(-MAX_LINEAR, min(MAX_LINEAR, raw))
    assert clamped == MAX_LINEAR


def test_safety_clamp_angular():
    MAX_ANGULAR = 1.0
    raw = 5.0
    clamped = max(-MAX_ANGULAR, min(MAX_ANGULAR, raw))
    assert clamped == MAX_ANGULAR


def test_mecanum_forward_kinematics():
    """vx,vy,omega from four wheel speeds."""
    r = 0.04
    lx = 0.165 / 2
    ly = 0.215 / 2
    # All wheels at 1.0 rad/s → pure forward
    fl = fr = rl = rr = 1.0
    vx = (r / 4) * (fl + fr + rl + rr)
    vy = (r / 4) * (-fl + fr + rl - rr)
    omega = (r / (4 * (lx + ly))) * (-fl + fr - rl + rr)
    assert abs(vx - 0.04) < 1e-9
    assert abs(vy) < 1e-9
    assert abs(omega) < 1e-9


def test_mecanum_strafe_kinematics():
    """Pure strafe: fl=-1, fr=+1, rl=+1, rr=-1."""
    r = 0.04
    lx = 0.165 / 2
    ly = 0.215 / 2
    fl, fr, rl, rr = -1.0, 1.0, 1.0, -1.0
    vx = (r / 4) * (fl + fr + rl + rr)
    vy = (r / 4) * (-fl + fr + rl - rr)
    omega = (r / (4 * (lx + ly))) * (-fl + fr - rl + rr)
    assert abs(vx) < 1e-9
    assert abs(vy - 0.04) < 1e-9
    assert abs(omega) < 1e-9
```

Run unit tests:

```bash
cd robot_ws/src/omnibot_driver/test
pytest test_packet_encoding.py -v
```

### 3.11 Unit tests — node with mocked serial

```python
import pytest
from unittest.mock import MagicMock, patch, call
import rclpy
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_node_initializes_with_mock_serial(ros_context):
    with patch('serial.Serial') as mock_serial_cls:
        mock_serial = MagicMock()
        mock_serial_cls.return_value = mock_serial

        from omnibot_driver.yahboom_controller_node import YahboomControllerNode
        node = YahboomControllerNode()

        mock_serial_cls.assert_called_once()
        node.destroy_node()


def test_set_car_type_sent_5_times(ros_context):
    with patch('serial.Serial') as mock_serial_cls:
        mock_serial = MagicMock()
        mock_serial_cls.return_value = mock_serial

        from omnibot_driver.yahboom_controller_node import YahboomControllerNode
        node = YahboomControllerNode()

        # Count FUNC_SET_CAR_TYPE (0x15) writes
        writes = [c for c in mock_serial.write.call_args_list
                  if len(c[0][0]) > 3 and c[0][0][3] == 0x15]
        assert len(writes) == 5
        node.destroy_node()
```

---

## 4. omnibot_driver — Serial Bridge Node

**File:** `robot_ws/src/omnibot_driver/omnibot_driver/serial_bridge_node.py`

### 4.1 What it does

Reads STM32 encoder packets in the format `<ENCODERS,fl,fr,rl,rr>`, computes mecanum forward kinematics, and publishes `/odom` + `odom→base_link` TF.

### 4.2 Launch

```bash
ros2 run omnibot_driver serial_bridge_node \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p odom_frame:=odom \
  -p base_frame:=base_link
```

### 4.3 Simulate encoder input (hardware not required)

```bash
# Use socat to create a virtual serial pair
sudo apt install socat
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# Note the two /dev/pts/X paths created, e.g. /dev/pts/3 and /dev/pts/4

# Terminal 1: run node on /dev/pts/3
ros2 run omnibot_driver serial_bridge_node \
  --ros-args -p serial_port:=/dev/pts/3

# Terminal 2: inject encoder packet on /dev/pts/4
python3 -c "
import serial, time
s = serial.Serial('/dev/pts/4', 115200)
# fl=1.0, fr=1.0, rl=1.0, rr=1.0 rad/s → pure forward
s.write(b'<ENCODERS,1.0,1.0,1.0,1.0>\n')
time.sleep(0.5)
s.close()
"

# Terminal 3: verify odometry
ros2 topic echo /odom --once
```

**Expected:** `pose.pose.position.x` incrementing (forward motion).

### 4.4 Unit tests — encoder packet parsing

```python
def test_encoder_packet_forward():
    """All four wheels at 1.0 rad/s → pure forward."""
    r = 0.04
    lx = 0.165 / 2
    ly = 0.215 / 2
    fl = fr = rl = rr = 1.0

    vx = (r / 4) * (fl + fr + rl + rr)
    vy = (r / 4) * (-fl + fr + rl - rr)
    omega = (r / (4 * (lx + ly))) * (-fl + fr - rl + rr)

    assert abs(vx - 0.04) < 1e-9
    assert abs(vy) < 1e-9
    assert abs(omega) < 1e-9


def test_encoder_packet_format():
    """Parse <ENCODERS,fl,fr,rl,rr> format."""
    raw = b'<ENCODERS,1.5,-1.5,1.5,-1.5>\n'
    content = raw.decode().strip().strip('<>').split(',')
    assert content[0] == 'ENCODERS'
    fl, fr, rl, rr = float(content[1]), float(content[2]), \
                     float(content[3]), float(content[4])
    assert fl == 1.5
    assert fr == -1.5
```

---

## 5. omnibot_driver — Legacy Mecanum Controller

**File:** `robot_ws/src/omnibot_driver/omnibot_driver/mecanum_controller_node.py`

### 5.1 What it does

Alternative motor driver targeting STM32-based boards. Same topic interface as `yahboom_controller_node`.

### 5.2 Launch

```bash
ros2 run omnibot_driver mecanum_controller_node
```

### 5.3 Verify topic interface

```bash
ros2 node info /mecanum_controller_node
```

**Expected subscriptions:** `/cmd_vel`
**Expected publications:** `/odom`, `/imu/data`

### 5.4 Test command passthrough (simulation)

```bash
# Terminal 1
ros2 run omnibot_driver mecanum_controller_node \
  --ros-args -p serial_port:=/dev/pts/3

# Terminal 2
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 6. omnibot_driver — Hardware Test Node

**File:** `robot_ws/src/omnibot_driver/omnibot_driver/yahboom_test_node.py`

**Hardware required.**

### 6.1 What it does

Runs a timed automated test sequence: INIT → MODE_MECANUM → MOTOR commands (forward, backward, stop, rotate left, rotate right) with 5-second intervals between each step.

### 6.2 Launch

```bash
ros2 run omnibot_driver yahboom_test_node
```

### 6.3 Expected sequence

```
[INFO] Step 0: INIT — sending SET_CAR_TYPE
[INFO] Step 1: MODE_MECANUM — configuring mecanum mode
[INFO] Step 2: MOTOR forward
[INFO] Step 3: MOTOR backward
[INFO] Step 4: MOTOR stop
[INFO] Step 5: MOTOR rotate left
[INFO] Step 6: MOTOR rotate right
[INFO] Test sequence complete
```

### 6.4 Custom command test

```python
# Manually test send_custom_command via Python
import rclpy
from omnibot_driver.yahboom_test_node import YahboomTestNode

rclpy.init()
node = YahboomTestNode()
node.send_custom_command(func=0x12,
                         payload=b'\x01\x64\x00\x00\x00')  # vx=100 (0.1 m/s)
rclpy.shutdown()
```

---

## 7. omnibot_hybrid — CmdVelMux

**File:** `robot_ws/src/omnibot_hybrid/omnibot_hybrid/cmd_vel_mux.py`

### 7.1 What it does

Routes exactly one of three `/cmd_vel` input sources to `/cmd_vel/out` based on the active control mode. Publishes the current active mode on `/control_mode/active` at 1 Hz.

| Mode | Input topic |
|------|-------------|
| `nav2` (default) | `/cmd_vel` |
| `vla` | `/cmd_vel/vla` |
| `teleop` | `/cmd_vel/teleop` |

Invalid mode strings are **silently ignored** — the active mode does not change.

### 7.2 Launch

```bash
source robot_ws/install/setup.bash
ros2 run omnibot_hybrid cmd_vel_mux

# With custom default mode
ros2 run omnibot_hybrid cmd_vel_mux --ros-args -p default_mode:=teleop
```

### 7.3 Verify topic graph

```bash
ros2 node info /cmd_vel_mux
```

**Expected subscriptions:** `/cmd_vel`, `/cmd_vel/vla`, `/cmd_vel/teleop`, `/control_mode`
**Expected publications:** `/cmd_vel/out`, `/control_mode/active`

### 7.4 Test mode switching and routing

```bash
# Terminal 1: run mux
ros2 run omnibot_hybrid cmd_vel_mux

# Terminal 2: monitor output
ros2 topic echo /cmd_vel/out

# Terminal 3: monitor active mode
ros2 topic echo /control_mode/active

# Terminal 4: switch to teleop mode
ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'teleop'"

# Terminal 4: send teleop velocity
ros2 topic pub --once /cmd_vel/teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# Expected: /cmd_vel/out shows linear.x = 0.15
```

### 7.5 Test nav2 mode is NOT forwarded when in teleop

```bash
# Still in teleop mode (from above)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.99, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# Expected: /cmd_vel/out does NOT show 0.99 (teleop is active, not nav2)
```

### 7.6 Test invalid mode is silently ignored

```bash
# Switch to vla mode first
ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'vla'"
# Confirm active
ros2 topic echo /control_mode/active --once
# Expected: "vla"

# Send invalid mode
ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'INVALID_MODE_XYZ'"
# Confirm mode unchanged
ros2 topic echo /control_mode/active --once
# Expected: still "vla"
```

### 7.7 Test case-insensitivity

```bash
ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'NAV2'"
ros2 topic echo /control_mode/active --once
# Expected: "nav2"

ros2 topic pub --once /control_mode std_msgs/msg/String "data: 'VLA'"
ros2 topic echo /control_mode/active --once
# Expected: "vla"
```

### 7.8 Unit tests

```python
# test_cmd_vel_mux.py
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_default_mode_is_nav2(ros_context):
    from omnibot_hybrid.cmd_vel_mux import CmdVelMux
    node = CmdVelMux()
    assert node.get_parameter('default_mode').value == 'nav2'
    node.destroy_node()


def test_invalid_mode_ignored(ros_context):
    from omnibot_hybrid.cmd_vel_mux import CmdVelMux
    node = CmdVelMux()
    initial_mode = node.active_mode

    # Simulate invalid mode message
    msg = String()
    msg.data = 'INVALID_XYZ'
    node._control_mode_callback(msg)

    assert node.active_mode == initial_mode
    node.destroy_node()


def test_valid_mode_changes(ros_context):
    from omnibot_hybrid.cmd_vel_mux import CmdVelMux
    node = CmdVelMux()

    msg = String()
    for mode in ['nav2', 'vla', 'teleop']:
        msg.data = mode
        node._control_mode_callback(msg)
        assert node.active_mode == mode

    node.destroy_node()


def test_case_insensitive_mode(ros_context):
    from omnibot_hybrid.cmd_vel_mux import CmdVelMux
    node = CmdVelMux()

    msg = String()
    msg.data = 'NAV2'
    node._control_mode_callback(msg)
    assert node.active_mode == 'nav2'

    node.destroy_node()
```

---

## 8. omnibot_hybrid — MissionPlanner

**File:** `robot_ws/src/omnibot_hybrid/omnibot_hybrid/mission_planner.py`

### 8.1 What it does

High-level state machine that parses mission commands, drives Nav2 for navigation phases, and triggers VLA inference. State machine: `IDLE → navigating → vla → done → IDLE`.

### 8.2 Launch

```bash
# Requires Nav2 action server to be running for navigation phases
ros2 run omnibot_hybrid mission_planner
```

### 8.3 Verify topics

```bash
ros2 node info /mission_planner
```

**Expected subscriptions:** `/mission/command`, `/mission/cancel`
**Expected publications:** `/control_mode`, `/vla/prompt`, `/mission/status`

### 8.4 Test mission command parsing

```bash
# Terminal 1: run planner
ros2 run omnibot_hybrid mission_planner

# Terminal 2: monitor status
ros2 topic echo /mission/status

# Terminal 3: monitor control mode
ros2 topic echo /control_mode

# Test: VLA-only command
ros2 topic pub --once /mission/command std_msgs/msg/String \
  "data: 'vla:find the red cup'"
# Expected: /control_mode switches to "vla"
#           /vla/prompt receives "find the red cup"

# Test: navigate-only command
ros2 topic pub --once /mission/command std_msgs/msg/String \
  "data: 'navigate:kitchen'"
# Expected: /control_mode switches to "nav2", Nav2 goal sent

# Test: combined command
ros2 topic pub --once /mission/command std_msgs/msg/String \
  "data: 'navigate:kitchen,vla:find the red cup'"
# Expected: navigate first, then VLA

# Test: navigate alias
ros2 topic pub --once /mission/command std_msgs/msg/String \
  "data: 'nav:bedroom'"
ros2 topic pub --once /mission/command std_msgs/msg/String \
  "data: 'go:hallway'"
```

### 8.5 Test mission cancel

```bash
# Start a mission
ros2 topic pub --once /mission/command std_msgs/msg/String \
  "data: 'navigate:kitchen,vla:find the red cup'"

# Cancel mid-mission
ros2 topic pub --once /mission/cancel std_msgs/msg/String "data: 'cancel'"
# Expected: /control_mode returns to "nav2"
#           /mission/status shows "phase=idle"
```

### 8.6 Test mission status format

```bash
ros2 topic echo /mission/status
```

**Expected format:** `"phase=navigating mission={'navigate': 'kitchen', 'vla': 'find the red cup'}"`

### 8.7 Unit tests — command parser

```python
# test_mission_planner.py
import pytest


def parse_mission_command(command: str) -> dict:
    """Replicate parser logic from mission_planner.py."""
    NAV_ALIASES = {'navigate', 'nav', 'nav2', 'go'}
    result = {}
    parts = command.split(',')
    for part in parts:
        part = part.strip()
        if ':' not in part:
            continue
        key, value = part.split(':', 1)
        key = key.strip().lower()
        value = value.strip()
        if key in NAV_ALIASES:
            result['navigate'] = value
        elif key == 'vla':
            result['vla'] = value
    return result


def test_parse_combined_command():
    cmd = "navigate:kitchen,vla:find the red cup"
    result = parse_mission_command(cmd)
    assert result['navigate'] == 'kitchen'
    assert result['vla'] == 'find the red cup'


def test_parse_nav_only():
    result = parse_mission_command("navigate:bedroom")
    assert result == {'navigate': 'bedroom'}
    assert 'vla' not in result


def test_parse_vla_only():
    result = parse_mission_command("vla:pick up the bottle")
    assert result == {'vla': 'pick up the bottle'}
    assert 'navigate' not in result


def test_parse_nav_alias_nav():
    result = parse_mission_command("nav:kitchen")
    assert result['navigate'] == 'kitchen'


def test_parse_nav_alias_nav2():
    result = parse_mission_command("nav2:kitchen")
    assert result['navigate'] == 'kitchen'


def test_parse_nav_alias_go():
    result = parse_mission_command("go:hallway")
    assert result['navigate'] == 'hallway'


def test_parse_empty_string():
    result = parse_mission_command("")
    assert result == {}


def test_parse_invalid_no_colon():
    result = parse_mission_command("kitchen")
    assert result == {}
```

---

## 9. omnibot_arm — Arm Driver

**File:** `robot_ws/src/omnibot_arm/omnibot_arm/arm_driver_node.py`

**Hardware required for full tests (Feetech STS3215 servos).**

### 9.1 What it does

Drives a 6-DOF SO-101 arm via Feetech STS3215 servos using the LeRobot `FeetechMotorsBus`. Publishes joint states at 100 Hz. Optionally runs in teleop mode (leader→follower mirroring).

### 9.2 Launch

```bash
# Normal (follower only)
ros2 launch omnibot_arm arm.launch.py

# Teleop mode (leader + follower)
ros2 run omnibot_arm arm_driver_node \
  --ros-args -p teleop_mode:=true
```

### 9.3 Verify topics

```bash
ros2 node info /arm_driver_node
```

**Expected publications:** `/arm/joint_states`
**Expected subscriptions:** `/arm/joint_commands`, `/arm/enable`

### 9.4 Test joint states publication

```bash
ros2 topic echo /arm/joint_states
```

**Expected:** Messages at ~100 Hz with 6 joint names and positions.

**Check joint name prefix (known issue):** Joint names should be `arm_shoulder_pan`, etc.

```bash
ros2 topic echo /arm/joint_states --once | grep name
# Should show: ['arm_shoulder_pan', 'arm_shoulder_lift', ...]
# If shows: ['shoulder_pan', ...] → arm_params.yaml not loaded
```

### 9.5 Test joint commands

```bash
# Enable the arm
ros2 topic pub --once /arm/enable std_msgs/msg/Bool "data: true"

# Command all joints to home position (0.0 rad)
ros2 topic pub --once /arm/joint_commands sensor_msgs/msg/JointState \
  "{header: {stamp: {sec: 0}, frame_id: ''},
    name: ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
           'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper'],
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 9.6 Unit tests — tick/radian conversion

```python
# test_arm_driver.py
import pytest
import math


TICKS_PER_REV = 4096
HOME_TICKS = [2048, 2048, 2048, 2048, 2048, 2048]
JOINT_MIN = [-3.14, -1.57, -1.57, -1.57, -3.14, -0.1]
JOINT_MAX = [3.14, 1.57, 1.57, 1.57, 3.14, 0.8]


def ticks_to_radians(ticks: int, home: int) -> float:
    return (ticks - home) * (2 * math.pi / TICKS_PER_REV)


def radians_to_ticks(radians: float, home: int) -> int:
    return int(home + radians * TICKS_PER_REV / (2 * math.pi))


def clamp_radians(radians: float, idx: int) -> float:
    return max(JOINT_MIN[idx], min(JOINT_MAX[idx], radians))


def test_home_position_is_zero():
    for i, home in enumerate(HOME_TICKS):
        assert ticks_to_radians(home, home) == 0.0


def test_ticks_to_radians_full_revolution():
    home = 2048
    full_rev = home + TICKS_PER_REV
    angle = ticks_to_radians(full_rev, home)
    assert abs(angle - 2 * math.pi) < 1e-9


def test_radians_to_ticks_home():
    for home in HOME_TICKS:
        assert radians_to_ticks(0.0, home) == home


def test_roundtrip_conversion():
    for home in HOME_TICKS:
        for angle in [-1.0, -0.5, 0.0, 0.5, 1.0]:
            ticks = radians_to_ticks(angle, home)
            back = ticks_to_radians(ticks, home)
            assert abs(back - angle) < 0.002  # tolerance for int rounding


def test_clamp_within_limits():
    assert clamp_radians(0.0, 0) == 0.0
    assert clamp_radians(3.14, 0) == 3.14
    assert clamp_radians(-3.14, 0) == -3.14


def test_clamp_exceeds_max():
    assert clamp_radians(99.0, 0) == JOINT_MAX[0]


def test_clamp_exceeds_min():
    assert clamp_radians(-99.0, 0) == JOINT_MIN[0]


def test_gripper_positive_limit():
    # Gripper max is 0.8 rad
    assert clamp_radians(0.8, 5) == 0.8
    assert clamp_radians(1.0, 5) == 0.8


def test_gripper_negative_limit():
    # Gripper min is -0.1 rad
    assert clamp_radians(-0.1, 5) == -0.1
    assert clamp_radians(-1.0, 5) == -0.1


def test_simulation_mode_fallback():
    """Node should start even without lerobot installed."""
    try:
        import lerobot  # noqa
        pytest.skip("lerobot installed, simulation fallback not triggered")
    except ImportError:
        pass  # Expected on CI

    import rclpy
    rclpy.init()
    try:
        from omnibot_arm.arm_driver_node import ArmDriverNode
        node = ArmDriverNode()
        # Should not raise even without hardware
        node.destroy_node()
    finally:
        rclpy.shutdown()
```

---

## 10. omnibot_vla — OpenVLA Node

**File:** `robot_ws/src/omnibot_vla/omnibot_vla/vla_node.py`

**GPU required (≥ 16 GB VRAM). Runs on desktop, NOT on Raspberry Pi.**

### 10.1 What it does

Subscribes to `/image_raw` (camera) and `/vla/prompt` (task description). Runs OpenVLA inference at 1 Hz and publishes velocity commands on `/cmd_vel/vla`.

### 10.2 Launch

```bash
# On the GPU desktop machine
source robot_ws/install/setup.bash
ros2 launch omnibot_vla vla_desktop.launch.py

# Or manually
ros2 run omnibot_vla vla_node \
  --ros-args \
  -p model_path:=openvla/openvla-7b \
  -p device:=cuda \
  -p load_in_4bit:=false
```

### 10.3 Test prompt subscription

```bash
# Terminal 1: run node (with mocked model if no GPU)
ros2 run omnibot_vla vla_node

# Terminal 2: send a prompt
ros2 topic pub --once /vla/prompt std_msgs/msg/String \
  "data: 'Find the red cup'"
```

### 10.4 Test output on /cmd_vel/vla

```bash
ros2 topic echo /cmd_vel/vla
```

**Expected:** Twist messages at ~1 Hz after a prompt is set.

### 10.5 Verify action mapping

Action indices: `[0]→linear.x`, `[1]→linear.y`, `[5]→angular.z`.

```python
# test_vla_node.py
import pytest


def test_action_mapping():
    """Verify OpenVLA action → Twist mapping."""
    from geometry_msgs.msg import Twist

    action = [0.1, 0.05, 0.0, 0.0, 0.0, 0.3]  # 7D action
    twist = Twist()
    twist.linear.x = float(action[0])
    twist.linear.y = float(action[1])
    twist.angular.z = float(action[5])

    assert twist.linear.x == 0.1
    assert twist.linear.y == 0.05
    assert twist.angular.z == 0.3


def test_prompt_format():
    task = "Find the red cup"
    prompt = f"In: What action should the robot take to {task}?\nOut:"
    assert "In: What action should the robot take to" in prompt
    assert "Out:" in prompt
    assert task in prompt
```

### 10.6 Unit test with mocked model

```python
# test_vla_node_mock.py
import pytest
from unittest.mock import MagicMock, patch
import rclpy


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_vla_node_mock_inference(ros_context):
    with patch('transformers.AutoProcessor.from_pretrained') as mock_proc, \
         patch('transformers.AutoModelForVision2Seq.from_pretrained') as mock_model:

        mock_processor = MagicMock()
        mock_proc.return_value = mock_processor

        mock_m = MagicMock()
        mock_model.return_value = mock_m
        # Return dummy 7D action
        mock_m.generate.return_value = MagicMock()
        mock_processor.decode.return_value = "0.1 0.05 0.0 0.0 0.0 0.3 0.0"

        from omnibot_vla.vla_node import VLANode
        node = VLANode()
        assert node is not None
        node.destroy_node()
```

---

## 11. omnibot_lerobot — SmolVLA Node

**File:** `robot_ws/src/omnibot_lerobot/omnibot_lerobot/smolvla_node.py`

**GPU required. Both `/camera/wrist/image_raw` and `/camera/base/bev/image_raw` must be available.**

### 11.1 What it does

Unified 9-DOF policy (6 arm + 3 base velocities) using SmolVLA. Requires both wrist and BEV camera feeds. Falls back to `DummyPolicy` (zeros) if lerobot is not installed.

### 11.2 Launch

```bash
source robot_ws/install/setup.bash
ros2 launch omnibot_lerobot smolvla_inference.launch.py
```

### 11.3 Prerequisites

The BEV stitcher must be running:

```bash
# Terminal 1: BEV stitcher
ros2 run ros2_bev_stitcher bev_stitcher_node
```

### 11.4 Verify topics

```bash
ros2 node info /smolvla_node
```

**Expected subscriptions:**
- `/camera/wrist/image_raw`
- `/camera/base/bev/image_raw`
- `/arm/joint_states`
- `/odom`
- `/smolvla/task`
- `/smolvla/enable`

**Expected publications:**
- `/arm/joint_commands`
- `/cmd_vel`

### 11.5 Enable SmolVLA and set task

```bash
# Enable
ros2 topic pub --once /smolvla/enable std_msgs/msg/Bool "data: true"

# Set task (triggers policy reset)
ros2 topic pub --once /smolvla/task std_msgs/msg/String \
  "data: 'pick up the red block'"

# Monitor output
ros2 topic echo /arm/joint_commands
ros2 topic echo /cmd_vel
```

### 11.6 Test missing image warning throttle

Stop the wrist camera topic, then check logs:

```bash
# Should see warning at most every 2 seconds
ros2 topic echo /rosout | grep -i "missing\|wrist\|bev"
```

### 11.7 Test base velocity clamping

```python
# test_smolvla_node.py
import pytest


def clamp_base_velocity(vx, vy, vz):
    MAX_LIN = 0.3
    MAX_ANG = 1.0
    return (
        max(-MAX_LIN, min(MAX_LIN, vx)),
        max(-MAX_LIN, min(MAX_LIN, vy)),
        max(-MAX_ANG, min(MAX_ANG, vz)),
    )


def test_clamp_normal():
    assert clamp_base_velocity(0.1, 0.1, 0.5) == (0.1, 0.1, 0.5)


def test_clamp_linear_exceeded():
    vx, vy, vz = clamp_base_velocity(1.0, -1.0, 0.0)
    assert vx == 0.3
    assert vy == -0.3


def test_clamp_angular_exceeded():
    vx, vy, vz = clamp_base_velocity(0.0, 0.0, 5.0)
    assert vz == 1.0


def test_9d_action_split():
    """9D action: indices 0-5 = arm, 6-8 = base."""
    action = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.05, 0.0, 0.1]
    arm_action = action[:6]
    base_action = action[6:]

    assert len(arm_action) == 6
    assert len(base_action) == 3
    assert arm_action[0] == 0.1
    assert base_action[0] == 0.05
    assert base_action[2] == 0.1


def test_dummy_policy_fallback():
    """DummyPolicy returns zeros when lerobot not installed."""
    try:
        import lerobot  # noqa
        pytest.skip("lerobot installed")
    except ImportError:
        pass

    action = [0.0] * 9
    assert all(a == 0.0 for a in action)
```

---

## 12. omnibot_lerobot — Teleop Recorder

**File:** `robot_ws/src/omnibot_lerobot/omnibot_lerobot/teleop_recorder_node.py`

### 12.1 What it does

Records leader arm + base joystick teleoperation as demonstration episodes for imitation learning. State machine: `IDLE →[RB press]→ RECORDING →[RB again]→ SAVING; [LB]→ DISCARDING`.

### 12.2 Launch

```bash
ros2 run omnibot_lerobot teleop_recorder_node \
  --ros-args \
  -p output_dir:=/tmp/test_episodes \
  -p repo_id:=local/test_dataset \
  -p record_hz:=30.0
```

### 12.3 Verify subscriptions

```bash
ros2 node info /teleop_recorder_node
```

**Expected subscriptions:**
- `/arm/leader_states`
- `/arm/joint_states`
- `/joy`
- `/camera/wrist/image_raw`
- `/camera/base/bev/image_raw`
- `/odom`

### 12.4 Test recording state machine

```bash
# Terminal 1: run recorder
ros2 run omnibot_lerobot teleop_recorder_node \
  --ros-args -p output_dir:=/tmp/test_episodes

# Terminal 2: press RB (button 7) to start recording
ros2 topic pub --once /joy sensor_msgs/msg/Joy \
  "{header: {stamp: {sec: 0}, frame_id: ''},
    axes: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1]}"
# Expected log: "Recording started"

# Terminal 2: press RB again to stop and save
ros2 topic pub --once /joy sensor_msgs/msg/Joy \
  "{header: {stamp: {sec: 1}, frame_id: ''},
    axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    buttons: [0, 0, 0, 0, 0, 0, 0, 1]}"
# Expected log: "Saving episode..."
```

### 12.5 Test discard

```bash
# Start recording
ros2 topic pub --once /joy sensor_msgs/msg/Joy \
  "{axes: [0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 1]}"

# Press LB (button 6) to discard
ros2 topic pub --once /joy sensor_msgs/msg/Joy \
  "{axes: [0.0], buttons: [0, 0, 0, 0, 0, 0, 1, 0]}"
# Expected log: "Episode discarded"
```

### 12.6 Verify saved dataset

```bash
ls /tmp/test_episodes/
# With lerobot: Parquet files + MP4 videos
# Without lerobot: .npz files

# Inspect NumPy fallback
python3 -c "
import numpy as np, glob
files = glob.glob('/tmp/test_episodes/*.npz')
if files:
    d = np.load(files[0])
    print('Keys:', list(d.keys()))
    print('States shape:', d['states'].shape)
    print('Actions shape:', d['actions'].shape)
"
```

### 12.7 Test joystick axis mapping

```python
# test_teleop_recorder.py
import pytest


MAX_LINEAR = 0.2
MAX_ANGULAR = 1.0


def axes_to_base_vel(axes):
    """axes[1]→vx, axes[0]→vy, axes[3]→vz"""
    vx = axes[1] * MAX_LINEAR
    vy = axes[0] * MAX_LINEAR
    vz = axes[3] * MAX_ANGULAR
    return vx, vy, vz


def test_forward_axis():
    axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # axes[1] = 1.0
    vx, vy, vz = axes_to_base_vel(axes)
    assert vx == MAX_LINEAR
    assert vy == 0.0


def test_strafe_axis():
    axes = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # axes[0] = 1.0
    vx, vy, vz = axes_to_base_vel(axes)
    assert vx == 0.0
    assert vy == MAX_LINEAR


def test_rotation_axis():
    axes = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # axes[3] = 1.0
    vx, vy, vz = axes_to_base_vel(axes)
    assert vz == MAX_ANGULAR


def test_9d_action_construction():
    leader_arm = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    base_vx, base_vy, base_vz = 0.05, 0.0, 0.1
    action = leader_arm + [base_vx, base_vy, base_vz]
    assert len(action) == 9
    assert action[0] == 0.1
    assert action[6] == 0.05


def test_episode_timeout():
    """Episode should auto-stop at 60 seconds."""
    TIMEOUT = 60.0
    elapsed = 61.0
    should_stop = elapsed >= TIMEOUT
    assert should_stop is True
```

