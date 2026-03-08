# SmolVLA Mobile Manipulation — Training Guide

Complete guide for collecting teleoperation data, converting it to training format, and fine-tuning SmolVLA on the unified mecanum base + SO-101 arm system.

---

## System Overview

The robot is treated as a **single 9-DOF entity**:

| Index | Name | Source | Range |
|-------|------|---------|-------|
| 0 | shoulder_pan | arm encoder | −π … +π rad |
| 1 | shoulder_lift | arm encoder | −π … +π rad |
| 2 | elbow_flex | arm encoder | −π … +π rad |
| 3 | wrist_flex | arm encoder | −π … +π rad |
| 4 | wrist_roll | arm encoder | −π … +π rad |
| 5 | gripper | arm encoder | 0 … 1 (normalised) |
| 6 | base_vx | `/odom` twist | −0.5 … +0.5 m/s |
| 7 | base_vy | `/odom` twist | −0.5 … +0.5 m/s |
| 8 | base_vz | `/odom` twist | −1.5 … +1.5 rad/s |

State = **observation at time t**. Action = **command sent at time t** (arm joint targets + base velocity). SmolVLA predicts `chunk_size = 50` future actions at once.

---

## 1. Hardware Setup

### 1.1 Camera Setup (Recommended)

You need **two cameras** for bimanual / mobile manipulation:

```
┌────────────────────────────────────────────────────────┐
│  Camera 1 — Front (workspace overview)                 │
│  • Logitech C920 / C270 (1080p / 720p)                 │
│  • Mount: front of robot chassis, ~30° downward tilt   │
│  • FOV: captures tabletop + arm workspace              │
│  • Topic: /camera/front/image_raw (640×480, 30 fps)    │
├────────────────────────────────────────────────────────┤
│  Camera 2 — Wrist (end-effector view)                  │
│  • Logitech C270 or any small USB cam                  │
│  • Mount: link_5 (wrist) of SO-101 arm                 │
│  • FOV: gripper + object close-up                      │
│  • Topic: /camera/wrist/image_raw (320×240, 30 fps)    │
└────────────────────────────────────────────────────────┘
```

**Why two cameras?**
- Front camera gives global context (where is the object, where is the robot).
- Wrist camera gives precise grasp context. SmolVLA uses both as separate visual tokens.

**Mounting tips:**
- Front cam: fixed bracket on robot front panel, at ~0.3 m height, angled down ~30°.
- Wrist cam: 3D-print a bracket that clamps to SO-101 link_5. Cable-tie the USB cable along the arm. Keep camera mass < 50 g.
- Avoid backlighting — task area lighting matters more than camera quality.

### 1.2 Teleop Hardware

| Device | Role |
|--------|------|
| Xbox / PS4 controller | Base teleoperation (left stick = vx/vy, right stick = vz) |
| SO-101 leader arm (`/dev/ttyACM1`) | Arm teleoperation (mirrors to follower) |
| SO-101 follower arm (`/dev/ttyACM0`) | Executes mirrored positions |

### 1.3 Verify Before Recording

```bash
# Check USB devices
ls /dev/ttyACM*        # should see ACM0 (follower) + ACM1 (leader)
ls /dev/video*         # should see video0 + video1 (or video2)

# Test cameras individually
ros2 run image_tools showimage --ros-args -r /image:=/camera/front/image_raw
ros2 run image_tools showimage --ros-args -r /image:=/camera/wrist/image_raw

# Verify arm driver is alive
ros2 topic echo /arm/joint_states --once

# Verify odom is publishing
ros2 topic echo /odom --once
```

---

## 2. Arm Calibration (one-time, when arm arrives)

```bash
# Install lerobot tooling
pip install "lerobot[feetech]"

# 1. Configure motor IDs on follower (run once per motor, motors ship as ID=1)
lerobot-setup-motors --robot-type=so101 --robot-port=/dev/ttyACM0 --mock

# 2. Calibrate both arms (move to requested positions, saves ~/.cache/lerobot/)
lerobot-calibrate --robot-type=so101 --robot-port=/dev/ttyACM0  # follower
lerobot-calibrate --robot-type=so101 --robot-port=/dev/ttyACM1  # leader

# 3. Update home_ticks in arm_params.yaml with values from calibration output
nano robot_ws/src/omnibot_arm/config/arm_params.yaml
```

---

## 3. Recording Teleoperation Episodes

### 3.1 Launch Full System

```bash
# Terminal 1 — bring up robot, arm, cameras
./launch_mobile_manipulation.sh

# Terminal 2 — launch teleop recorder node
ros2 launch omnibot_lerobot teleop_record.launch.py \
    output_dir:=/data/episodes \
    task_name:="pick_and_place_cube" \
    fps:=30
```

### 3.2 Recording Controls (Xbox Controller)

| Button | Action |
|--------|--------|
| **RB** (right bumper) | **Toggle recording ON/OFF** |
| **LB** (left bumper) | **Discard current episode** (keeps robot running) |
| Left stick | Base vx / vy (strafe) |
| Right stick X | Base vz (rotation) |
| Left stick click | Emergency stop |

**Leader arm** is always live-mirroring to follower — just move it naturally.

### 3.3 What Gets Recorded Per Frame

The `teleop_recorder_node` captures at `fps` Hz and stores:

```
state  [9D]: current arm joint angles + base odom velocities
action [9D]: arm commands sent + base cmd_vel issued this step
obs/front_image  : JPEG (640×480)
obs/wrist_image  : JPEG (320×240)
timestamp        : ROS time in nanoseconds
```

Each episode is saved as a **ROS 2 bag** under:
```
/data/episodes/
  pick_and_place_cube_ep000/
  pick_and_place_cube_ep001/
  ...
```

### 3.4 Episode Quality Guidelines

| Metric | Target |
|--------|--------|
| Episode length | 5–20 seconds (150–600 frames @ 30 fps) |
| Episodes per task | ≥ 50 (100+ for robust policies) |
| Task variation | Vary object position ±15 cm, orientation ±45°, lighting |
| Success rate | Record only **successful** episodes — abort with LB if failed |

---

## 4. Converting Bags → Unified Dataset

### 4.1 Install Data Engine

```bash
cd data_engine
pip install -e .
pip install -r requirements.txt
```

### 4.2 Single Bag → HDF5

```bash
python ingestion/bag_to_hdf5.py \
    --bag /data/episodes/pick_and_place_cube_ep000 \
    --output /data/hdf5/pick_and_place_cube_ep000.h5 \
    --topics \
        /arm/joint_states \
        /arm/joint_commands \
        /odom \
        /cmd_vel \
        /camera/front/image_raw \
        /camera/wrist/image_raw \
    --fps 30
```

### 4.3 Batch Convert All Episodes

```bash
python scripts/ingest_dataset.py \
    --input-dir /data/episodes \
    --output-dir /data/hdf5 \
    --workers 4 \
    --fps 30
```

### 4.4 Convert HDF5 → LeRobot HF Format

LeRobot's training pipeline expects Parquet + video files in a specific layout. Use the provided conversion script:

```bash
python lerobot_engine/convert_hdf5_to_lerobot.py \
    --hdf5-dir /data/hdf5 \
    --output-dir /data/lerobot_dataset/pick_and_place_cube \
    --task-name "pick_and_place_cube" \
    --chunk-size 50
```

> **Note:** `convert_hdf5_to_lerobot.py` needs to be written (see Section 4.5 below).

### 4.5 What the Conversion Script Must Do

The LeRobot HF dataset format requires:

```
/data/lerobot_dataset/pick_and_place_cube/
  meta/
    info.json          ← dataset metadata (fps, action/state dims, num_episodes)
    tasks.jsonl        ← one line per task {"task_index":0,"task":"pick_and_place_cube"}
    episodes.jsonl     ← one line per episode {"episode_index":N,"tasks":[0],"length":T}
    stats.json         ← per-feature mean/std/min/max for normalisation
  data/
    chunk-000/
      episode_000000.parquet
      episode_000001.parquet
      ...
  videos/
    chunk-000/
      observation.images.front/
        episode_000000.mp4
      observation.images.wrist/
        episode_000000.mp4
```

Each Parquet row has columns:
```
observation.state          float32[9]
action                     float32[9]
observation.images.front   str  (path to video frame — handled by lerobot internally)
observation.images.wrist   str
timestamp                  float64  (seconds from episode start)
frame_index                int64
episode_index              int64
index                      int64  (global frame index)
task_index                 int64
next.done                  bool
```

### 4.6 Validate the Dataset

```bash
python scripts/validate_dataset.py --dataset /data/hdf5 --verbose

# Check a specific episode visually
python visualization/visualize_episode.py \
    --file /data/hdf5/pick_and_place_cube_ep000.h5 \
    --episode 0
```

---

## 5. Training SmolVLA

### 5.1 Prerequisites

```bash
cd lerobot_engine
pip install -r requirements.txt
# Requires: torch>=2.1, CUDA 12+, ~8 GB VRAM minimum (fp16)
# Recommended: RTX 3080 / 4070 or better, or a cloud GPU (Colab A100, RunPod)
```

### 5.2 Run Fine-Tuning

```bash
python train.py \
    --dataset-path /data/lerobot_dataset/pick_and_place_cube \
    --config configs/smolvla_mobile_manip.yaml \
    --output-dir /data/checkpoints/smolvla_manip_v1 \
    --epochs 100 \
    --batch-size 16 \
    --lr 1e-4 \
    --wandb-project omnibot-smolvla   # optional, requires wandb login
```

### 5.3 Key Config Parameters (`configs/smolvla_mobile_manip.yaml`)

```yaml
policy:
  chunk_size: 50          # predict 50 future steps at once
  n_action_steps: 50      # execute all 50 before re-querying
  state_dim: 9            # 6 arm + 3 base
  action_dim: 9

normalization:
  # These ranges must match your actual robot's operating envelope
  state_min:  [-3.14, -3.14, -3.14, -3.14, -3.14, 0.0, -0.5, -0.5, -1.5]
  state_max:  [ 3.14,  3.14,  3.14,  3.14,  3.14, 1.0,  0.5,  0.5,  1.5]
  action_min: [-3.14, -3.14, -3.14, -3.14, -3.14, 0.0, -0.5, -0.5, -1.5]
  action_max: [ 3.14,  3.14,  3.14,  3.14,  3.14, 1.0,  0.5,  0.5,  1.5]
```

### 5.4 Training on a Cloud GPU (if no local GPU)

```bash
# Option A: HuggingFace Spaces / AutoTrain (easiest)
# Upload /data/lerobot_dataset/ to HF Hub, then use lerobot CLI:
huggingface-cli upload <your-org>/omnibot-pick-place /data/lerobot_dataset/

lerobot train \
    --policy.type=smolvla \
    --dataset.repo_id=<your-org>/omnibot-pick-place \
    --output_dir=outputs/smolvla_v1 \
    --training.num_epochs=100

# Option B: Google Colab / RunPod — rsync dataset + run train.py
```

### 5.5 Monitoring Training

```
outputs/smolvla_v1/
  checkpoints/
    epoch_010/model.pt
    epoch_020/model.pt
    ...
    best/model.pt        ← saved when val loss improves
  train_log.json
```

Key metrics to watch:
- **action_loss** (L1 on predicted vs actual actions) — should decrease steadily
- **val_action_loss** — if this diverges from train loss, you need more data or more regularisation
- Target: val_action_loss < 0.05 for reliable policy

---

## 6. Running Inference

### 6.1 Update Checkpoint Path

```yaml
# robot_ws/src/omnibot_lerobot/config/smolvla_params.yaml
smolvla_node:
  ros__parameters:
    checkpoint_path: "/data/checkpoints/smolvla_manip_v1/best/model.pt"
    task_description: "pick up the red cube and place it in the bin"
    action_scale_base: 1.0
    enable_on_start: false
```

### 6.2 Launch Inference

```bash
# Terminal 1 — full system
./launch_mobile_manipulation.sh

# Terminal 2 — SmolVLA inference
ros2 launch omnibot_lerobot smolvla_inference.launch.py

# Terminal 3 — enable the policy (robot will start moving)
ros2 topic pub /smolvla/enable std_msgs/Bool "data: true" --once

# To change task at runtime:
ros2 topic pub /smolvla/task std_msgs/String \
    "data: 'pick up the blue block'" --once
```

### 6.3 Safety During Inference

- Keep Xbox controller in hand — **left stick click = emergency stop** via `/emergency_stop`
- First test with `action_scale_base: 0.3` (30% base speed) until confident
- Run in a clear 2m × 2m area

---

## 7. Iterative Improvement

```
Record 50 demos → Train → Test → Identify failures
         ↑                              |
         └──── Record 20 more demos ←──┘
               focused on failure cases
```

### 7.1 Dataset Expansion

```bash
# Add new episodes to existing dataset
python scripts/ingest_dataset.py \
    --input-dir /data/episodes_v2 \
    --output-dir /data/hdf5 \
    --workers 4

# Re-run LeRobot conversion (will append new episodes)
python lerobot_engine/convert_hdf5_to_lerobot.py \
    --hdf5-dir /data/hdf5 \
    --output-dir /data/lerobot_dataset/pick_and_place_cube \
    --task-name "pick_and_place_cube" \
    --chunk-size 50

# Resume training from checkpoint
python lerobot_engine/train.py \
    --dataset-path /data/lerobot_dataset/pick_and_place_cube \
    --config configs/smolvla_mobile_manip.yaml \
    --output-dir /data/checkpoints/smolvla_manip_v2 \
    --resume-from /data/checkpoints/smolvla_manip_v1/best/model.pt \
    --epochs 50
```

### 7.2 Multi-Task Training

To train on multiple tasks at once, merge datasets:

```bash
python lerobot_engine/convert_hdf5_to_lerobot.py \
    --hdf5-dir /data/hdf5 \
    --output-dir /data/lerobot_dataset/multi_task \
    --task-name "pick_and_place_cube" \
    --chunk-size 50

# Add second task to same dataset dir
python lerobot_engine/convert_hdf5_to_lerobot.py \
    --hdf5-dir /data/hdf5_task2 \
    --output-dir /data/lerobot_dataset/multi_task \   # same output
    --task-name "push_button" \
    --chunk-size 50
```

---

## 8. File Reference

| File | Purpose |
|------|---------|
| `lerobot_engine/record.py` | Standalone recorder (no ROS, for bench testing) |
| `lerobot_engine/train.py` | Fine-tune SmolVLA on collected dataset |
| `lerobot_engine/infer.py` | Standalone inference test (no robot) |
| `lerobot_engine/configs/smolvla_mobile_manip.yaml` | Policy config (dims, normalisation, hyperparams) |
| `robot_ws/src/omnibot_lerobot/omnibot_lerobot/teleop_recorder_node.py` | ROS 2 recorder (live robot) |
| `robot_ws/src/omnibot_lerobot/omnibot_lerobot/smolvla_node.py` | ROS 2 inference node |
| `robot_ws/src/omnibot_arm/config/arm_params.yaml` | Arm motor IDs, ports, home ticks |
| `data_engine/ingestion/bag_to_hdf5.py` | ROS bag → HDF5 conversion |
| `data_engine/ingestion/ros_parser.py` | Low-level bag parsing (cameras, odom, cmd_vel) |
| `data_engine/ingestion/sync_topics.py` | Multi-modal timestamp synchronisation |
| `data_engine/schema/constants.py` | 9D state/action spec definitions |
| `data_engine/schema/format.py` | HDF5 group/dataset schema |
| `data_engine/loader/dataset.py` | PyTorch Dataset over HDF5 files |
| `data_engine/visualization/visualize_episode.py` | Playback episodes with OpenCV overlay |

---

## 9. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Arm jitters during inference | `action_scale_arm` too high | Lower to 0.5 in `smolvla_params.yaml` |
| Base drifts unexpectedly | Odom not calibrated | Check wheel diameter in `yahboom_params.yaml` |
| `chunk_size` mismatch error | Config vs checkpoint mismatch | Ensure same `chunk_size=50` at record + train + infer |
| Low val accuracy despite many demos | Insufficient task variation | Vary object position, lighting, start pose |
| GPU OOM during training | Batch too large | Reduce `batch_size` to 8, enable `fp16: true` |
| Camera topics missing at record time | USB cam not detected | Check `ls /dev/video*`, update device in `mobile_manipulation.launch.py` |
| HDF5 validation fails (`frame_count mismatch`) | Recording dropped frames | Reduce `fps` to 15 or improve USB bandwidth (dedicated USB controller) |
