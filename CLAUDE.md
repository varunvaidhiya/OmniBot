# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OmniBot is a ROS 2 mecanum wheel robot with embodied AI capabilities (OpenVLA). The repo is a monorepo with:
- `robot_ws/` — ROS 2 workspace (Jazzy, Ubuntu 24.04)
- `vla_engine/` — Standalone PyTorch training/inference for VLA models
- `data_engine/` — Episode-based dataset collection pipeline
- `android_app/` — Kotlin MVVM Android controller (ROSBridge WebSocket)
- `infra/` — Docker, DevContainers, CI/CD scripts

## Build & Test

```bash
# Build all ROS 2 packages
cd robot_ws
colcon build --symlink-install
source install/setup.bash

# Run all tests
colcon test
colcon test-result --verbose

# Run tests for a single package
colcon test --packages-select omnibot_driver
colcon test-result --verbose --test-result-base build/omnibot_driver
```

## Launch Commands

```bash
# Basic robot (driver + state publisher)
ros2 launch omnibot_bringup robot.launch.py

# Robot + Xbox controller (combined)
ros2 launch omnibot_bringup robot_with_joy.launch.py

# Xbox controller teleoperation only
ros2 launch omnibot_bringup joy_teleop.launch.py

# Gazebo simulation + RViz (convenience script handles build + source)
./launch_simulation.sh
# or directly:
ros2 launch omnibot_bringup simulation.launch.py

# Autonomous navigation with SLAM
ros2 launch omnibot_navigation autonomous_robot.launch.py

# VLA inference (requires NVIDIA GPU, 16GB+ VRAM)
ros2 launch omnibot_vla vla_desktop.launch.py

# Send a VLA prompt
ros2 topic pub --once /vla/prompt std_msgs/msg/String "data: 'Find the red cup'"
```

## Architecture

### ROS 2 Packages (`robot_ws/src/`)

| Package | Role |
|---|---|
| `omnibot_bringup` | Launch files, RViz config, Gazebo bridge config |
| `omnibot_driver` | Motor control — Yahboom board (primary) and STM32 (legacy) |
| `omnibot_navigation` | SLAM (slam_toolbox), Nav2, EKF localization, waypoint navigation |
| `omnibot_vla` | OpenVLA ROS 2 node — wraps `vla_engine/` |
| `omnibot_description` | URDF/xacro, meshes |
| `omnibot_firmware` | Legacy STM32 firmware (not actively developed) |

### Key Data Flow

```
Android/Xbox Controller
  → ROS 2 Network (Raspberry Pi 5)
      ├── yahboom_controller_node (/cmd_vel → serial → motors, /odom ← encoders)
      ├── slam_toolbox (mapping)
      ├── nav2_bringup (autonomous planning)
      └── data_logger (episode collection)

VLA Engine (Desktop GPU PC, separate ROS node)
  → subscribes /image_raw + /vla/prompt
  → publishes /cmd_vel
```

### Yahboom Serial Protocol (`confirmed_protocol.py`)

The Rosmaster serial protocol was reverse-engineered and is now the source of truth:

```
Packet: [0xFF, 0xFC, LEN, FUNC, PAYLOAD..., CHECKSUM]
  LEN      = 1 (ID) + 1 (LEN) + 1 (FUNC) + N (payload bytes)
  CHECKSUM = (sum(all_packet_bytes) + 5) & 0xFF  // 5 = 257 - 0xFC
```

Key function codes:
| Constant | Value | Purpose |
|---|---|---|
| `FUNC_BEEP` | `0x02` | Buzzer control |
| `FUNC_MOTOR` | `0x10` | Direct wheel velocity |
| `FUNC_MOTION` | `0x12` | Holonomic motion (vx, vy, vz ×1000, little-endian int16) |
| `FUNC_SET_CAR_TYPE` | `0x15` | Set robot type (X3 = 1) |

The root-level `*.py` files (`fuzz_*.py`, `scan_*.py`, `test_*.py`, `analyze_*.py`, etc.) are hardware debugging scripts used during protocol reverse-engineering — not part of the ROS package.

### Motor Control Node (`omnibot_driver/scripts/yahboom_controller_node.py`)

- Subscribes `/cmd_vel` (Twist) and `/joy`
- Publishes `/odom` + broadcasts TF
- 10 Hz control loop with velocity ramping
- Serial: `/dev/ttyUSB0` at 115200 baud (ROS parameter: `serial_port`, `baud_rate`)
- Debug log written to `/home/varunvaidhiya/yahboom_debug.log` on robot
- Mecanum kinematics also in `omnibot_driver/src/mecanum_controller.cpp`
- Wheel params: radius=0.04m, x-separation=0.165m, y-separation=0.215m

### Gazebo Simulation

- Uses `ros_gz_sim` + `ros_gz_bridge` (Gazebo Harmonic)
- Bridges `/cmd_vel`, `/odom`, `/tf` between ROS and Gazebo
- `ROS_DOMAIN_ID=30` aligns simulation traffic with physical robot domain

### VLA Integration

- ROS node entry point: `omnibot_vla/omnibot_vla/vla_node.py`
- Standalone ML engine: `vla_engine/` (PyTorch, no ROS dependency)
- Dataset format: episode-based (not rosbags) — see `data_engine/`

## CI/CD

GitHub Actions (`.github/workflows/ros2_ci.yml`) runs on Ubuntu 24.04 with ROS 2 Jazzy:
- Serial library pulled via FetchContent (not system package)
- Linting tests are **disabled** (test deps commented out in package.xml)
- NumPy pinned to `<2.0` for VLA compatibility

## Hardware Notes

- **Robot brain**: Raspberry Pi 5 8GB
- **AI brain**: Desktop PC with NVIDIA GPU (16GB+ VRAM)
- **Motor board**: Yahboom ROS Robot Expansion Board (replaces legacy STM32)
- See `MIGRATION_GUIDE.md` for STM32 → Yahboom migration details
