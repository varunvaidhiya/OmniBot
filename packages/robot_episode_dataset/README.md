# robot_episode_dataset

> LeRobot-compatible episode dataset library for robot imitation learning.
> ROS 2 bag → synchronized frames → Parquet + MP4 → PyTorch Dataset.

## Features

- **`TopicSynchronizer`** — nearest-neighbour multi-modal timestamp sync
  (no ROS dependency, works on raw timestamp arrays)
- **`EpisodeDataset`** — PyTorch-compatible loader for LeRobot v2.0 format
- **`DatasetSchema`** — configurable state/action/camera specification
- Ready-made OmniBot schema (`OMNIBOT_SCHEMA`) included as an example
- Pure Python, no ROS dependency for loading

## Install

```bash
pip install robot-episode-dataset

# With PyTorch support
pip install "robot-episode-dataset[torch]"

# With OpenCV (for video frame loading)
pip install "robot-episode-dataset[viz]"
```

## Usage

### Load a dataset

```python
from robot_episode_dataset import EpisodeDataset

ds = EpisodeDataset(
    root='/data/my_robot_episodes',
    image_keys=['observation.images.bev', 'observation.images.wrist'],
)

print(len(ds))        # number of frames
sample = ds[0]        # dict with state, action, images, timestamp
```

### Synchronize multi-modal data

```python
from robot_episode_dataset import TopicSynchronizer

sync = TopicSynchronizer(target_fps=10.0, sync_tolerance=0.05)

frames = sync.synchronize({
    'camera_bev':   [(ts_ns, img), ...],   # from /camera/base/bev/image_raw
    'camera_wrist': [(ts_ns, img), ...],   # from /camera/wrist/image_raw
    'state':        [(ts_ns, state), ...],
    'action':       [(ts_ns, action), ...],
})
# frames: list of dicts, each containing all modalities at one timestep
```

### Define your own schema

```python
from robot_episode_dataset import DatasetSchema, StateSpec, ActionSpec, CameraConfig

schema = DatasetSchema(
    robot_name='my_robot',
    state_spec=StateSpec(
        names=['joint_0', 'joint_1', 'vx'],
        ranges=[(-3.14, 3.14), (-3.14, 3.14), (-1.0, 1.0)],
        units=['rad', 'rad', 'm/s'],
    ),
    action_spec=ActionSpec(
        names=['joint_0', 'joint_1', 'vx'],
        ranges=[(-3.14, 3.14), (-3.14, 3.14), (-1.0, 1.0)],
        units=['rad', 'rad', 'm/s'],
    ),
    cameras=[
        CameraConfig('front', '/camera/front/image_raw', (480, 640), 30),
    ],
)
```

## Dataset Format (LeRobot v2.0)

```
<root>/
  meta/
    info.json          # Dataset metadata
    tasks.jsonl        # Task descriptions
    episodes.jsonl     # Per-episode metadata
    stats.json         # Normalization statistics
  data/
    chunk-000/
      episode_000000.parquet   # state, action, timestamp, frame_index
  videos/
    chunk-000/
      observation.images.bev/episode_000000.mp4
      observation.images.wrist/episode_000000.mp4
```

## License

Apache-2.0
