# OmniBot — Low-Cost Mobile Manipulation with SmolVLA

> **A $500 mobile manipulation robot** that learns pick-and-place tasks from ~50 human demonstrations.
> Mecanum base + SO-101 arm + 6-camera array controlled by a single learned policy.
> Native Android app with 3D SceneView robot viewer, AI natural-language control, real-time SLAM mapping, and dataset recording. Fully open-source.

![OmniBot demo 1](assets/Omnibot_demo1.gif)
![OmniBot demo 2](assets/Omnibot_demo2.gif)

<p align="center">
  <a href="../../wiki/Bill-of-Materials"><img src="https://img.shields.io/badge/BOM-Wiki-blue"/></a>
  <img src="https://img.shields.io/badge/ROS_2-Jazzy-brightgreen"/>
  <img src="https://img.shields.io/badge/Policy-SmolVLA-orange"/>
  <img src="https://img.shields.io/badge/Android-Kotlin-purple"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow"/>
</p>

---

## Why OmniBot?

| | OmniBot | Mobile ALOHA | Hello Stretch |
|--|--|--|--|
| **Cost** | **~$500** | ~$32,000 | ~$25,000 |
| **Base type** | **Holonomic (mecanum)** | Differential | Differential |
| **Policy** | SmolVLA (unified) | ACT | Various |
| **Cameras** | **6 (BEV array + depth + wrist)** | 3 | 2 |
| **Open-source** | Yes | Yes | Yes |

**Three things that make it different:**

- **Holonomic base** — strafes, rotates in place, moves diagonally without repositioning. Differential drive robots can't do this.
- **Unified 9D policy** — SmolVLA controls the arm (6 joints) and base (vx, vy, vz) as a single output vector. One model, one action space, one training run.
- **6-camera array** — 4× wide-angle base cameras → Bird's Eye View composite + rear depth camera for obstacle avoidance + wrist camera for grasp precision.

---

## Android App

A native Kotlin MVVM controller app that connects over ROSBridge WebSocket (`ws://robot:9090`).

### App Demo

<!-- TODO: Record demo video and replace placeholders with actual GIFs -->

**Dashboard — Live camera, velocity, odometry**

![Dashboard screen](assets/app_dashboard.gif)
<!-- Placeholder: record gif showing camera feed, BEV overlay toggle, live velocity charts, odometry readout -->

**AI Chat — Natural language robot control**

![AI chat screen](assets/app_ai_chat.gif)
<!-- Placeholder: record gif typing "go to the kitchen and pick up the red cup", robot executing mission -->

**2D SLAM Map — Real-time occupancy grid**

![SLAM map screen](assets/app_slam_map.gif)
<!-- Placeholder: record gif showing robot building map in real time, pose overlay updating -->

**3D Point Cloud — Depth camera visualisation**

![Point cloud screen](assets/app_point_cloud.gif)
<!-- Placeholder: record gif showing live PointCloud2 from Orbbec, pinch-to-zoom, Jet colormap -->

**3D Robot Viewer — Live animated model**

![3D robot viewer screen](assets/app_robot_viewer.gif)
<!-- Placeholder: record gif showing robot.glb in SceneView, arm joints moving, tap-to-navigate -->

**Controls — Joystick + arm panel**

![Controls screen](assets/app_controls.gif)
<!-- Placeholder: record gif showing virtual joystick driving robot, arm sliders moving servos -->

---

### App Feature Summary

| Feature | Status | Notes |
|---------|--------|-------|
| ROSBridge WebSocket connection | ✅ | Auto-reconnect, exponential back-off, 5 retries |
| Connection status indicator | ✅ | Green/orange/red dot, top-right all screens |
| Live camera feed (MJPEG) | ✅ | `web_video_server`, Content-Length + JPEG-marker parser |
| Camera fullscreen dialog | ✅ | Tap fullscreen button; dark overlay with close btn |
| BEV map overlay on camera | ✅ | Toggle BEV icon; robot footprint on camera card |
| Dataset recording button | ✅ | REC badge in camera card; publishes to `record_start/stop` |
| Dark mode | ✅ | Force-dark Night mode; `values-night/colors.xml` |
| OmniBot logo in toolbar | ✅ | Custom robot-head vector drawable |
| **2D SLAM map tab** | ✅ | Full-screen `SlamMapView` with robot pose overlay |
| SLAM map — robot pose panel | ✅ | X / Y / θ live readout, bottom-left |
| SLAM map — map stats panel | ✅ | Resolution, explored area m², occupied cells, bottom-right |
| SLAM map — zoom hint | ✅ | Pinch/pan hint fades out after 3 s |
| **3D Point Cloud tab** | ✅ | Canvas-based `PointCloudView`, Jet colormap depth render |
| Point cloud — pinch to zoom | ✅ | ScaleGestureDetector, min/max zoom clamped |
| Point cloud — stats overlay | ✅ | Point count, max range, render FPS |
| **3D Robot Viewer tab** | ✅ | SceneView 2.2.1 (Google Filament), loads `robot.glb` |
| Robot viewer — GLB model | ✅ | Named nodes match URDF link names; `tools/urdf_to_glb.py` |
| Robot viewer — live pose | ✅ | Model moves on floor plane from `/odom` |
| Robot viewer — arm animation | ✅ | Per-joint rotation applied from `/arm/joint_states` |
| Robot viewer — tap to navigate | ✅ | Ray-cast vs y=0 plane → Nav2 goal published |
| Robot viewer — no-model banner | ✅ | Instructions shown when `robot.glb` not in assets |
| **AI chat interface** | ✅ | "Hi Varun, what do you want me to do?" greeting |
| AI chat — natural language commands | ✅ | Routes to `mission_planner` via `/mission/command` |
| AI chat — quick chips | ✅ | One-tap: Navigate Home / Pick Up Object / Return to Dock |
| AI chat — mission status updates | ✅ | Robot replies streamed from `/mission/status` |
| AI chat — dataset recording toggle | ✅ | Start/stop recording from chat panel |
| Velocity charts (vx / vy / vz) | ✅ | Rolling 60-point line chart, bottom of Dashboard |
| Odometry readout | ✅ | X / Y / θ on Dashboard |
| Bottom navigation (5 tabs) | ✅ | Dashboard / Map / AI / Controls / Settings |
| Hilt dependency injection | ✅ | All ViewModels inject `RobotRepository` singleton |
| Coroutines + StateFlow | ✅ | All live data as cold-to-hot StateFlow |

---

### Navigation Structure

```
MainActivity
├── Dashboard      — camera feed, BEV overlay, velocity chart, odometry
├── Map            — ViewPager2 (3 sub-tabs)
│   ├── 2D SLAM    — OccupancyGrid map with live robot pose
│   ├── 3D Cloud   — PointCloud2 depth visualisation
│   └── 3D Robot   — SceneView GLB model + tap-to-navigate
├── AI             — natural language chat + dataset recording
├── Controls       — virtual joystick + arm joint sliders
└── Settings       — robot IP, port, camera URL
```

---

### 3D Robot Viewer Setup

The 3D Robot tab loads `robot.glb` from `android_app/app/src/main/assets/`. This file is not
checked in (large binary). Generate it from the URDF + STL meshes:

```bash
pip install trimesh[easy] numpy lxml
python tools/urdf_to_glb.py
# Output: android_app/app/src/main/assets/robot.glb
```

The script expands `omnibot.urdf.xacro`, loads all STL meshes from
`robot_ws/src/omnibot_description/meshes/`, applies joint origin transforms,
and exports a GLB where each node is named after the URDF link (e.g.
`arm_shoulder_pan`). The Android fragment finds nodes by name and applies live
joint rotations from `/arm/joint_states`.

If `robot.glb` is absent the viewer still shows live pose and joint angles as
text overlays, and shows setup instructions.

---

### Dataset Recording (from the App)

Tap the **REC** button on the Dashboard camera card or use the AI chat recording
toggle. This publishes to `/record_start` and `/record_stop` (std_msgs/Bool).

On the robot side, start the recorder node first:

```bash
ros2 launch omnibot_lerobot teleop_record.launch.py \
    output_dir:=/data/episodes \
    task_name:="pick up the red cube"
```

The recorder will begin writing when it receives the `/record_start` signal.

---

## Hardware

See the **[Bill of Materials →](../../wiki/Bill-of-Materials)** for full parts list, suppliers, and prices.

### Platform

| Component | Model | Notes |
|-----------|-------|-------|
| Robot brain | Raspberry Pi 5 8GB | Runs all ROS 2 nodes |
| AI brain | Desktop PC, NVIDIA GPU (16GB+ VRAM) | SmolVLA training + inference |
| Motor board | Yahboom ROS Robot Expansion Board | Mecanum drive + encoder feedback |
| Arm | SO-101 6-DOF | Feetech STS3215 servos |
| IMU | MPU9250 (on Yahboom board) | 100 Hz accel + gyro |

### Sensor Array (6 cameras)

```
                        ┌─────────────────────────────────┐
                        │  TOP PLATE                      │
                        │                                 │
   [Orbbec Astra Pro]◄──┤  rear-centre, 12° down-tilt    │
   depth + RGB, 0.6-8m  │  facing REARWARD                │
                        │                                 │
                        │  [SO-101 Arm]                   │
                        │      │                          │
                        │      └── wrist_camera_link      │
                        │          OV9732, 100° FOV       │
                        └─────────────────────────────────┘
                        ┌─────────────────────────────────┐
                        │  BOTTOM PLATE                   │
    [OV9732 LEFT]  ◄────┤  ←  left edge, facing left     │
    [OV9732 RIGHT] ◄────┤  →  right edge, facing right   │
    [OV9732 FRONT] ◄────┤  ↑  front edge, facing forward │
    [OV9732 REAR]  ◄────┤  ↓  rear edge, facing rearward │
                        └─────────────────────────────────┘
```

| Camera | Model | Position | Topic | Purpose |
|--------|-------|----------|-------|---------|
| Front | OV9732, 100° FOV | Bottom plate front edge | `/camera/front/image_raw` | BEV input |
| Rear | OV9732, 100° FOV | Bottom plate rear edge | `/camera/rear/image_raw` | BEV input |
| Left | OV9732, 100° FOV | Bottom plate left edge | `/camera/left/image_raw` | BEV input |
| Right | OV9732, 100° FOV | Bottom plate right edge | `/camera/right/image_raw` | BEV input |
| Depth | **Orbbec Astra Pro** | Top plate, rear-centre | `/camera/depth/image_raw` `/camera/depth/points` | Nav2 costmap, obstacle avoidance |
| Wrist | OV9732, 100° FOV | SO-101 gripper (link_5) | `/camera/wrist/image_raw` | Grasp precision, SmolVLA |

**BEV composite:** the 4 base cameras are stitched by `ros2_bev_stitcher` into a single 800×800 top-down image on `/camera/base/bev/image_raw`. SmolVLA uses BEV + wrist as its two visual inputs.

**Depth camera design note:** The Orbbec Astra Pro faces rearward because Nav2 drives the robot backwards toward the workspace. The 12° downward tilt ensures the floor in front of the arm is in the depth FOV.

---

## Quickstart

### 1. Install dependencies

```bash
# ROS 2 Jazzy + colcon
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization ros-jazzy-usb-cam

# Orbbec Astra Pro ROS 2 driver
sudo apt install ros-jazzy-orbbec-camera   # or build from source:
# https://github.com/orbbec/OrbbecSDK_ROS2

# Python deps
pip install "lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git"
cd data_engine && pip install -e .
```

### 2. Build

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch full system

```bash
# Everything: base + arm + all 6 cameras + BEV stitcher
./launch_mobile_manipulation.sh

# Xbox controller teleoperation
ros2 launch omnibot_bringup joy_teleop.launch.py

# Android app connectivity (required — port 9090)
./launch_rosbridge.sh
```

### 4. Verify sensors

```bash
# Check all 6 camera topics are live
ros2 topic hz /camera/front/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/wrist/image_raw
ros2 topic hz /camera/base/bev/image_raw

# Check depth pointcloud
ros2 topic hz /camera/depth/points

# Check arm joints
ros2 topic echo /arm/joint_states --once

# Check odometry
ros2 topic echo /odom --once
```

### 5. Record teleoperation episodes

```bash
ros2 launch omnibot_lerobot teleop_record.launch.py \
    output_dir:=/data/episodes \
    task_name:="pick up the red cube"
# Xbox RB = start/stop  |  LB = discard
# Leader arm mirrors to follower arm in real time
```

### 6. Convert and train

```bash
python -m data_engine.scripts.ingest_dataset \
    --input-dir /data/episodes \
    --output-dir /data/lerobot/pick_place \
    --task "pick up the red cube"

python lerobot_engine/train.py \
    --dataset-path /data/lerobot/pick_place \
    --output-dir /data/checkpoints/smolvla_v1
```

### 7. Run inference

```bash
ros2 launch omnibot_lerobot smolvla_inference.launch.py
ros2 topic pub /smolvla/enable std_msgs/Bool "data: true" --once
ros2 topic pub /smolvla/task std_msgs/String "data: 'pick up the red cube'" --once
```

---

## Architecture

```
Teleop hardware
  ├── Xbox controller  → base (vx, vy, vz)
  └── SO-101 leader arm → follower arm (6 joints)
            │
            ▼
   teleop_recorder_node  →  ROS 2 bags
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
                    ┌────────────┴────────────┐
           /arm/joint_commands          /cmd_vel
                 │                         │
           SO-101 arm                Mecanum base
         (6 STS3215 servos)      (Yahboom serial → 4 wheels)

Android App ──ROSBridge ws://robot:9090──► ROS 2 Network
  Natural language chat → /mission/command → mission_planner
  Tap-to-navigate (SceneView) → /mission/command (navigate:pose:x,y)
  Dataset REC button → /record_start, /record_stop
```

### Unified 9D action / state space

```
[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper,
 base_vx,      base_vy,       base_vz]
  ◄────────────── arm (rad) ──────────────►  ◄──── base (m/s, rad/s) ────►
```

SmolVLA visual inputs: **BEV image** (800×800, from 4 base cameras) + **wrist image** (640×480).

### ROS 2 packages

| Package | Role |
|---------|------|
| `omnibot_bringup` | Launch files, RViz config, Gazebo bridge |
| `omnibot_driver` | Yahboom serial driver, mecanum kinematics, odometry |
| `omnibot_arm` | SO-101 arm driver (FeetechMotorsBus, 100 Hz joint states) |
| `omnibot_lerobot` | SmolVLA inference node, teleop recorder, BEV stitcher |
| `omnibot_hybrid` | cmd_vel multiplexer, mission planner (teleop ↔ Nav2 ↔ VLA) |
| `omnibot_navigation` | SLAM (slam_toolbox + RTAB-Map), Nav2, EKF localization |
| `omnibot_description` | URDF/xacro, SO-101 STL meshes |
| `omnibot_vla` | Legacy OpenVLA node |

### Standalone packages (`packages/`)

| Package | Role |
|---------|------|
| `yahboom_ros2` | Publishable Yahboom driver (pip/bloom) |
| `mecanum_drive_ros2` | Standalone mecanum kinematics (C++ + Python) |
| `ros2_bev_stitcher` | Bird's Eye View homography stitcher |
| `robot_episode_dataset` | LeRobot-compatible dataset loader |
| `rosbridge-android` | Android ROSBridge Gradle library |
| `vla_serve` | OpenVLA inference HTTP server + Docker |

---

## Simulation

```bash
./launch_simulation.sh
# Gazebo Harmonic + RViz
# All 6 sensors simulated: 4 base cameras, depth RGBD, wrist camera
```

---

## Autonomous Navigation

```bash
# SLAM + Nav2 (holonomic config — allows lateral motion)
ros2 launch omnibot_navigation autonomous_robot.launch.py

# RTAB-Map 3D SLAM (uses Orbbec depth + odometry)
ros2 launch omnibot_navigation rtabmap.launch.py

# Hybrid: navigate to location → VLA task
ros2 launch omnibot_hybrid hybrid_robot.launch.py
# Send a mission:
ros2 topic pub /mission/command std_msgs/String \
    "data: 'navigate:kitchen,vla:pick up the red cup'" --once
```

---

## Training Guide

See **[data_engine/TRAINING_GUIDE.md](data_engine/TRAINING_GUIDE.md)** for the full end-to-end workflow:
- Camera mounting and calibration
- Recording 50+ teleoperation episodes
- ROS bag → LeRobot dataset conversion
- SmolVLA fine-tuning (local GPU or cloud)
- Iterative improvement loop

---

## Repository Structure

```
├── robot_ws/src/
│   ├── omnibot_driver/         # Yahboom serial driver, odometry
│   ├── omnibot_arm/            # SO-101 arm driver
│   ├── omnibot_lerobot/        # SmolVLA inference, recorder, BEV stitcher
│   ├── omnibot_hybrid/         # cmd_vel mux, mission planner
│   ├── omnibot_navigation/     # SLAM + Nav2
│   ├── omnibot_bringup/        # Launch files
│   └── omnibot_description/    # URDF + SO-101 STL meshes
├── packages/
│   ├── yahboom_ros2/           # Standalone publishable driver
│   ├── mecanum_drive_ros2/     # Standalone kinematics
│   ├── ros2_bev_stitcher/      # BEV stitcher
│   ├── robot_episode_dataset/  # Dataset loader
│   ├── rosbridge-android/      # Android Gradle library
│   └── vla_serve/              # VLA HTTP inference server
├── android_app/                # Kotlin MVVM controller app (5-tab navigation)
├── tools/
│   └── urdf_to_glb.py          # URDF + STL → robot.glb for Android SceneView
├── data_engine/                # ROS bag → LeRobot v2.0 dataset pipeline
├── lerobot_engine/             # SmolVLA train / infer scripts
├── vla_engine/                 # Legacy OpenVLA stack
└── infra/                      # Docker, CI/CD
```

---

## Completed Features

### Robot / ROS 2

- [x] Mecanum base with working odometry
- [x] ROS 2 Nav2 holonomic configuration
- [x] RTAB-Map 3D SLAM with Orbbec depth
- [x] SO-101 arm: 100 Hz joint states, teleop + follower mode
- [x] 6-camera array (4× base + depth + wrist)
- [x] Bird's Eye View stitcher (800×800 composite)
- [x] SmolVLA unified 9D policy node
- [x] LeRobot v2.0 dataset pipeline
- [x] cmd_vel multiplexer (teleop / Nav2 / VLA modes)
- [x] Mission planner (`navigate:X,vla:Y` command parser)
- [x] ROSBridge WebSocket server integration
- [x] Gazebo Harmonic simulation (all 6 sensors)
- [x] AI orchestration layer (LangGraph + LangSmith + CrewAI + AutoGen)

### Android App

- [x] ROSBridge connection with auto-reconnect
- [x] Live MJPEG camera stream
- [x] Camera fullscreen + BEV overlay toggle
- [x] Dataset recording trigger (REC button)
- [x] Dark mode (force-night theme)
- [x] OmniBot logo in toolbar
- [x] 2D SLAM map with robot pose + stats overlays
- [x] 3D Point Cloud viewer (Jet colormap, pinch-to-zoom)
- [x] 3D Robot Viewer (SceneView / Filament, tap-to-navigate)
- [x] Live arm joint animation in 3D viewer
- [x] AI chat interface ("Hi Varun, what do you want me to do?")
- [x] Natural language → mission command routing
- [x] Quick-action chips (Navigate Home / Pick Up Object / Return to Dock)
- [x] Mission status replies in chat
- [x] Velocity charts (vx / vy / vz)
- [x] Odometry overlay

---

## Future Work

### High Priority

- [ ] **Emergency stop wiring** — Android publishes `/emergency_stop` (Bool) but no ROS node subscribes. Add subscriber to `yahboom_controller_node.py`.
- [ ] **Launch file parameter mismatch** — `robot.launch.py` passes `wheel_separation_x/y` but node declares `wheel_separation_length/width`. Must be aligned (see CLAUDE.md Known Issues #1).
- [ ] **Arm joint name prefix** — `arm_driver_node.py` default `joint_names` omits `arm_` prefix. Always override via `arm_params.yaml` (Known Issues #2).
- [ ] **`omnibot_recorder` ROS node** — A dedicated ROS node to respond to `/record_start` + `/record_stop` topics and write LeRobot-format datasets (currently must start `teleop_record.launch.py` manually).
- [ ] **First trained SmolVLA checkpoint** — Record 50+ episodes and train; upload to HuggingFace.

### Android App

- [ ] **ARCore mode** — Augmented reality overlay of robot model on live camera feed using ARCore + SceneView AR session.
- [ ] **Foxglove debug panel** — Embedded Foxglove WebView for accessing raw topic streams and plots during debugging.
- [ ] **Voice input** — Android SpeechRecognizer feeding the AI chat interface.
- [ ] **Multi-camera switcher** — Swipe between wrist / front / BEV camera feeds on Dashboard.
- [ ] **Unit tests for RobotRepository** — Refactor singleton to constructor DI for proper unit testing.

### Infrastructure

- [ ] **CI GLB generation** — GitHub Actions step to run `tools/urdf_to_glb.py` and upload `robot.glb` as a release asset.
- [ ] **Coverage thresholds** — Enable `ament_lint_auto` and add `pytest --cov` minimum 60% gate.
- [ ] **Demo video** — Record end-to-end: app connection → AI command → robot navigation + pick-and-place.
- [ ] **HuggingFace dataset + model upload**

---

## Phase Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| 1 — Hardware + Base | ✅ Done | Mecanum base, Yahboom serial, odometry, Nav2 |
| 2 — Arm + 6-Camera | ✅ Done | SO-101, BEV stitcher, SmolVLA node |
| 3 — Android App | ✅ Done | MVVM app, ROSBridge, all 5 tabs, dark mode |
| 4 — AI Control | ✅ Done | Natural language chat, AI orchestration layer, 3D viewer |
| 5 — Learning | 🔄 In progress | Record 50 episodes, train SmolVLA, evaluate |

---

## License

MIT

---

## Acknowledgments

- [HuggingFace LeRobot](https://github.com/huggingface/lerobot) — SmolVLA policy and SO-101 tooling
- [SceneView](https://github.com/SceneView/sceneview-android) — Google Filament 3D rendering for Android
- [Orbbec](https://www.orbbec.com) — Astra Pro depth camera and ROS 2 SDK
- [Yahboom](https://www.yahboom.com) — ROS Robot Expansion Board
- [Foxglove](https://foxglove.dev) — Robot visualization tooling
- ROS 2 community
