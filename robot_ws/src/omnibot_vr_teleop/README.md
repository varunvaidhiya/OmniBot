# omnibot_vr_teleop

VR teleoperation node — **contribution welcome**.

## Overview

This package is a placeholder for a VR teleoperation interface for OmniBot.
The intended design is:

- **Subscribe**: `/camera/front/image_raw` and `/camera/wrist/image_raw` (streamed to VR headset)
- **Publish**: `/cmd_vel/teleop` (`geometry_msgs/Twist`) for base motion
- **Publish**: `/arm/joint_commands` (`sensor_msgs/JointState`) for arm control

Set `/control_mode` to `"teleop"` before driving the base via VR.

## Suggested Approach

A WebXR-based approach works well:
1. Stream camera images via `web_video_server` or `foxglove_bridge`
2. Read headset pose and controller input via WebXR API in the browser
3. Bridge commands back to ROS via ROSBridge WebSocket (`ws://robot:9090`)

Alternatively, implement a native ROS 2 node using an OpenXR binding.

## Contributing

If you build a VR teleoperation node, please open a pull request!
See [CONTRIBUTING.md](../../../CONTRIBUTING.md) for guidelines.

Joint names for arm control must use the `arm_` prefix:
```
arm_shoulder_pan, arm_shoulder_lift, arm_elbow_flex,
arm_wrist_flex, arm_wrist_roll, arm_gripper
```
