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
