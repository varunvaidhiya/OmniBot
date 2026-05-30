# OmniBot Open

**Hardware kit + teleoperation stack for a mecanum-wheel mobile-manipulation robot.**

OmniBot Open gives you everything you need to build the robot, drive it, and
collect high-quality teleoperation demonstrations for imitation learning —
without any autonomous navigation, AI inference, or cloud dependency.

---

## What's Included

| Area | Contents |
|---|---|
| **Hardware description** | URDF, xacro, meshes (`omnibot_description`) |
| **Yahboom base driver** | Serial driver for the Yahboom expansion board (`omnibot_driver`) |
| **SO-101 arm driver** | 6-DOF arm via LeRobot FeetechMotorsBus (`omnibot_arm`) |
| **Xbox teleoperation** | Joy node + teleop_twist_joy (`joy_teleop.launch.py`) |
| **Android app** | Full Kotlin MVVM app — camera view, arm control, emergency stop |
| **VR teleoperation** | Placeholder package (`omnibot_vr_teleop`) — contribution welcome |
| **LeRobot data recording** | Leader-follower imitation data collection (`teleop_recorder_node`) |
| **BEV stitcher** | 4-camera bird's-eye-view for wrist + overhead data collection |
| **Yahboom protocol library** | Pure-Python encoder/decoder (`packages/yahboom_ros2`) |
| **Mecanum kinematics** | C++17 + Python kinematics library (`packages/mecanum_drive_ros2`) |

## What's NOT Included

The following are available in **OmniBot Pro** (private):

- Autonomous navigation (Nav2, SLAM, EKF)
- VLA / AI inference (OpenVLA, SmolVLA)
- Reinforcement learning (Isaac Lab, ONNX policies)
- AI orchestration (LangGraph, Claude)
- Gazebo / Isaac Sim digital twin

---

## Hardware BOM

> **TODO**: Add a full Bill of Materials here.

Key components:
- Yahboom ROS Robot Expansion Board (mecanum drive)
- 4× mecanum wheels (60 mm radius)
- Raspberry Pi 5 (8 GB) as robot brain
- SO-101 6-DOF arm with Feetech STS3215 servos (LeRobot compatible)
- 5× USB cameras (4 base-mounted + 1 wrist)
- Orbbec Astra Pro RGB-D camera (optional, for depth)
- Xbox controller (for teleoperation)

See `robot_ws/src/omnibot_description/urdf/` for the full URDF with link
dimensions and joint placements.

---

## Assembly Guide

> **TODO**: Add step-by-step hardware assembly instructions with photos.

Reference the URDF at `robot_ws/src/omnibot_description/urdf/omnibot.urdf.xacro`
for link geometry and joint limits. The firmware package
(`robot_ws/src/omnibot_firmware/`) contains legacy STM32 notes — the active
motor board is the Yahboom expansion board via USB serial.

---

## Quick Start

### 1. Build

```bash
# Clone and build
git clone https://github.com/your-org/omnibot-open
cd omnibot-open

source /opt/ros/jazzy/setup.bash
cd robot_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

### 2. Configure deployment

```bash
# Copy and edit the deployment template
cp deployment.env.example deployment.env
# Edit deployment.env: set YAHBOOM_PORT, robot IP, etc.

python deploy.py              # interactive menu
python deploy.py --mode single  # non-interactive single-machine
```

### 3. Xbox teleoperation

```bash
# Launch robot driver + Xbox controller
ros2 launch omnibot_bringup robot_with_joy.launch.py

# Or use the convenience script (sources network.env for multi-machine DDS):
./launch_teleop.sh
```

Xbox controls (default `config/xbox_teleop.yaml`):
- **Enable**: RB (button 5) — hold to drive
- **Turbo**: RT (button 7) — faster speed
- **Left stick**: linear X/Y
- **Right stick**: angular Z

### 4. Robot + Android app

```bash
# Start ROSBridge WebSocket server (port 9090)
./launch_rosbridge.sh
# Or: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

Connect the Android app to `ws://<robot-ip>:9090`. See [Android App](#android-app) below.

---

## Data Collection

Use `teleop_recorder_node` to record leader-follower demonstrations in
[LeRobot HuggingFace dataset format](https://github.com/huggingface/lerobot).

```bash
# Launch the recorder (requires arm driver + cameras to be running)
ros2 launch omnibot_lerobot teleop_record.launch.py

# On the Xbox controller:
#   RB  — start recording an episode
#   LB  — discard current episode
# Episodes auto-save after timeout or on RB release.
```

Key parameters (edit `config/smolvla_params.yaml`):

| Parameter | Default | Description |
|---|---|---|
| `output_dir` | `~/datasets/mobile_manipulation` | Dataset output path |
| `repo_id` | `local/mobile_manipulation` | LeRobot repo ID |
| `record_hz` | `30.0` Hz | Recording frequency |
| `episode_timeout_s` | `60.0` s | Max episode length |

The BEV stitcher (`bev_stitcher_node`) must be running to provide the
`/camera/base/bev/image_raw` overhead view for data collection:

```bash
ros2 launch omnibot_bringup perception.launch.py
```

---

## Android App

The Android app provides manual teleoperation, arm joint control, camera feeds,
and emergency stop via ROSBridge WebSocket.

**Connection**: `ws://<robot-ip>:9090`  
**Default IP**: `192.168.1.100` — change in the app settings.

**Controls:**
- Virtual joystick → `/cmd_vel/teleop` (base motion)
- Arm sliders → `/arm/joint_commands`
- Emergency stop button → `/emergency_stop`
- Camera feed → MJPEG via `web_video_server`

Before driving, the app sends `/control_mode = "teleop"` to route base commands
through the mux to the Yahboom driver.

---

## VR Teleoperation

VR teleoperation is a **placeholder** — no implementation exists yet.

The `robot_ws/src/omnibot_vr_teleop/` package contains a README describing
the expected interface:
- **Subscribe**: `/camera/front/image_raw`, `/camera/wrist/image_raw`
- **Publish**: `/cmd_vel/teleop`, `/arm/joint_commands`

A WebXR + ROSBridge approach is recommended. Contributions welcome — see
[CONTRIBUTING.md](CONTRIBUTING.md).

---

## Multi-Machine Networking

Edit `network.env` with your machine IPs before running cross-machine:

```bash
WORKSTATION_IP=192.168.1.100   # desktop / data collection PC
PI_IP=192.168.1.101            # Raspberry Pi 5 running ROS
```

`launch_teleop.sh` sources this file and sets `ROS_STATIC_PEERS` automatically.
`ROS_DOMAIN_ID=30` must match on all machines.

---

## Contributing

Contributions are especially welcome for:

- **Hardware docs**: assembly photos, BOM, CAD files
- **VR teleoperation**: implement `omnibot_vr_teleop` (see its README)
- **New teleoperation methods**: phone, web UI, spacemouse
- **Camera calibration**: intrinsics + extrinsics for BEV stitcher
- **Tests**: driver unit tests, kinematics tests

Please open an issue before starting large features. See [CONTRIBUTING.md](CONTRIBUTING.md)
for code style and PR guidelines.

---

## License

Apache 2.0 — see [LICENSE](LICENSE).

Copyright 2024 OmniBot Contributors.
