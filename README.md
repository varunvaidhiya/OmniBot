# OmniBot — Low-Cost Mobile Manipulation with SmolVLA

> **A $500 mobile manipulation robot** that learns pick-and-place tasks from ~50 human demonstrations.
> Mecanum base + SO-101 arm controlled by a single learned policy. Fully open-source.

<!-- Replace with your demo GIF/video once arm arrives -->
<!-- ![OmniBot demo](docs/assets/demo.gif) -->

<p align="center">
  <a href="../../wiki/Bill-of-Materials"><img src="https://img.shields.io/badge/BOM-Wiki-blue"/></a>
  <img src="https://img.shields.io/badge/ROS_2-Jazzy-brightgreen"/>
  <img src="https://img.shields.io/badge/Policy-SmolVLA-orange"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow"/>
</p>

---

## Why OmniBot?

| | OmniBot | Mobile ALOHA | Hello Stretch |
|--|--|--|--|
| **Cost** | **~$500** | ~$32,000 | ~$25,000 |
| **Base type** | **Holonomic (mecanum)** | Differential | Differential |
| **Policy** | SmolVLA (unified) | ACT | Various |
| **Open-source** | Yes | Yes | Yes |
| **ROS 2** | Yes | Partial | Yes |

**Three things that make it different:**

- **Holonomic base** — strafes, rotates in place, moves diagonally. No repositioning maneuvers before grasping. Differential drive robots can't do this.
- **Unified 9D policy** — SmolVLA controls the arm (6 joints) and base (vx, vy, vz) as a single output vector. One model, one action space, one training run.
- **End-to-end open pipeline** — teleoperation recording → LeRobot v2.0 dataset → fine-tuned SmolVLA → ROS 2 inference. Every piece is in this repo.

---

## Hardware

See the **[Bill of Materials →](../../wiki/Bill-of-Materials)** for parts, suppliers, and prices.

**Key components:**
- Raspberry Pi 5 8GB (robot brain)
- Yahboom ROS Robot Expansion Board (motor driver + encoders)
- SO-101 Arm — 6-DOF, Feetech STS3215 servos (~$100)
- 2× USB cameras (front workspace + wrist)
- Xbox controller (base teleoperation)
- Desktop PC with NVIDIA GPU (SmolVLA training + inference)

---

## Quickstart

### 1. Build the ROS 2 workspace

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch the robot

```bash
# Base + arm + cameras
./launch_mobile_manipulation.sh

# Base only (before arm arrives)
ros2 launch omnibot_bringup robot.launch.py

# Xbox controller teleoperation
ros2 launch omnibot_bringup joy_teleop.launch.py
```

### 3. Connect the Android app

```bash
./launch_rosbridge.sh
# Open app → Settings → enter robot IP
```

### 4. Record teleoperation episodes

```bash
ros2 launch omnibot_lerobot teleop_record.launch.py \
    output_dir:=/data/episodes \
    task_name:="pick up the red cube"
# Xbox RB = start/stop recording | LB = discard episode
```

### 5. Convert and train

```bash
# ROS bags → LeRobot v2.0 dataset
python -m data_engine.scripts.ingest_dataset \
    --input-dir /data/episodes \
    --output-dir /data/lerobot/pick_place \
    --task "pick up the red cube"

# Fine-tune SmolVLA
python lerobot_engine/train.py \
    --dataset-path /data/lerobot/pick_place \
    --output-dir /data/checkpoints/smolvla_v1
```

### 6. Run inference

```bash
ros2 launch omnibot_lerobot smolvla_inference.launch.py
ros2 topic pub /smolvla/enable std_msgs/Bool "data: true" --once
```

---

## Architecture

```
Xbox controller + SO-101 leader arm
  │  (teleoperation)
  ▼
teleop_recorder_node  ──→  ROS 2 bags
                                │
                    bag_to_omnibot.py
                                │
                    LeRobot v2.0 dataset
                    (Parquet + MP4 video)
                                │
                      lerobot_engine/train.py
                                │
                    SmolVLA checkpoint
                                │
                       smolvla_node.py
                       /arm/joint_commands
                       /cmd_vel
                      ↙           ↘
              SO-101 arm      Mecanum base
```

**Unified 9D action space:**
```
[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper,
 base_vx, base_vy, base_vz]
```

### ROS 2 Packages

| Package | Role |
|---------|------|
| `omnibot_bringup` | Launch files |
| `omnibot_driver` | Yahboom serial driver, odometry, mecanum kinematics |
| `omnibot_arm` | SO-101 arm driver (FeetechMotorsBus, 100 Hz) |
| `omnibot_lerobot` | SmolVLA inference node + teleop recorder |
| `omnibot_navigation` | SLAM, Nav2 (holonomic config) |
| `omnibot_description` | URDF / xacro |
| `omnibot_vla` | Legacy OpenVLA node |

---

## Simulation

```bash
./launch_simulation.sh
# Gazebo Harmonic + RViz, bridges /cmd_vel /odom /tf
```

---

## Training Guide

See **[data_engine/TRAINING_GUIDE.md](data_engine/TRAINING_GUIDE.md)** for:
- Camera mounting recommendations
- Recording best practices (50+ episodes, vary object position)
- Full dataset conversion pipeline
- SmolVLA fine-tuning on local GPU or cloud (Colab / RunPod)
- Iterative improvement loop

---

## Repository Structure

```
├── robot_ws/src/
│   ├── omnibot_driver/       # Motor control, odometry
│   ├── omnibot_arm/          # SO-101 arm driver
│   ├── omnibot_lerobot/      # SmolVLA inference + recorder
│   ├── omnibot_navigation/   # SLAM + Nav2
│   ├── omnibot_bringup/      # Launch files
│   └── omnibot_description/  # URDF
├── data_engine/              # ROS bag → LeRobot dataset pipeline
├── lerobot_engine/           # SmolVLA train / infer scripts
├── vla_engine/               # Legacy OpenVLA stack
├── android_app/              # Kotlin MVVM controller app
└── infra/                    # Docker, CI
```

---

## Roadmap

- [x] Mecanum base with working odometry
- [x] ROS 2 Nav2 holonomic configuration
- [x] Android app with ROSBridge
- [x] SO-101 arm driver (simulation mode until arm arrives)
- [x] SmolVLA unified 9D policy node
- [x] LeRobot v2.0 dataset pipeline
- [ ] First real arm teleoperation episode
- [ ] First trained SmolVLA checkpoint
- [ ] Demo video
- [ ] HuggingFace dataset + model upload

---

## License

MIT

---

## Acknowledgments

- [HuggingFace LeRobot](https://github.com/huggingface/lerobot) — SmolVLA policy and SO-101 tooling
- [Yahboom](https://www.yahboom.com) — ROS Robot Expansion Board
- ROS 2 community
