# ros2_bev_stitcher

> Multi-camera **Bird's Eye View** compositor for ROS 2.
> The only open-source ROS 2 package for real-time ground-plane camera fusion.

Subscribes to N perspective cameras, warps each frame onto a shared top-down
canvas via per-camera homography matrices, alpha-blends overlapping regions,
and publishes a single unified BEV image — ready for VLA training, navigation,
or operator display.

## Features

- Configurable camera list (works with 1–8+ cameras)
- Per-camera homography-based perspective warp (OpenCV `warpPerspective`)
- Distance-weighted alpha blending in overlap zones
- **One-command calibration tool** (`bev_calibrate`) using a flat checkerboard
- Graceful fallback to tiled layout when uncalibrated
- All parameters exposed as ROS 2 params — no code changes needed

## Quick Start

```bash
# 1. Build
cd ~/ros2_ws/src && ln -s /path/to/Mecanum-Wheel-Robot/packages/ros2_bev_stitcher .
cd ~/ros2_ws && colcon build --packages-select ros2_bev_stitcher
source install/setup.bash

# 2. Calibrate (one-time, ~10 min)
#    Place a 9×6 checkerboard flat on the ground, visible to all cameras.
ros2 run ros2_bev_stitcher bev_calibrate \
    --camera-names front rear left right \
    --square-size 0.025 \
    --output ~/bev_calibration.npz

# 3. Launch
ros2 launch ros2_bev_stitcher bev_stitcher.launch.py

# 4. View result
ros2 run rqt_image_view rqt_image_view
# → select /camera/bev/image_raw
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `camera_names` | `['front','rear','left','right']` | Camera names |
| `input_topic_pattern` | `/camera/{name}/image_raw` | Topic pattern |
| `output_topic` | `/camera/bev/image_raw` | Published BEV topic |
| `canvas_size` | `800` | Internal compositing canvas (px, square) |
| `output_width/height` | `800` | Final image dimensions |
| `src_width/height` | `640/480` | Input image dimensions |
| `publish_hz` | `30.0` | Publish rate (Hz) |
| `calibration_file` | `~/bev_calibration.npz` | Homography file |
| `output_frame_id` | `bev_frame` | TF frame in image header |

## Calibration

The `bev_calibrate` tool detects checkerboard corners in each camera, maps
them to a shared ground-plane coordinate system, and computes a 3×3 RANSAC
homography per camera.

```bash
ros2 run ros2_bev_stitcher bev_calibrate --help
```

## Algorithm

```
For each camera:
  img → warpPerspective(H) → warped_canvas
  blend_weight = distance-to-edge map warped through H

Final BEV = Σ(warped_i × weight_i) / Σ(weight_i)   (per pixel)
```

## License

Apache-2.0
