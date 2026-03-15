# mecanum_drive_ros2

> Generic mecanum wheel kinematics library for ROS 2 — hardware-agnostic,
> works with any motor board.

## Features

- **C++17 header-only** library (`mecanum_kinematics.hpp`) — drop into any C++ ROS 2 package
- **Pure Python** mirror (`mecanum_drive_ros2.kinematics`) — use in nodes, tests, notebooks
- Inverse kinematics: body twist → wheel angular velocities
- Forward kinematics: wheel angular velocities → body twist
- Pose integration helper with heading wrap to [-π, π]
- Parameterized geometry — configure for any mecanum robot

## Kinematics

```
Wheel layout:         Sign conventions (ROS REP 103):
  FL ─── FR             vx  = forward  (+x)
  │       │             vy  = left     (+y)
  BL ─── BR             ω   = CCW      (+z)

Inverse kinematics:
  FL = (vx - vy - (L+W)·ω) / r
  FR = (vx + vy + (L+W)·ω) / r
  BL = (vx + vy - (L+W)·ω) / r
  BR = (vx - vy + (L+W)·ω) / r
```

## Usage — Python

```python
from mecanum_drive_ros2 import RobotGeometry, inverse_kinematics, forward_kinematics

geom = RobotGeometry(
    wheel_radius=0.04,
    wheel_separation_width=0.215,
    wheel_separation_length=0.165,
)

# Inverse kinematics
fl, fr, bl, br = inverse_kinematics(vx=0.3, vy=0.0, omega=0.0, geom=geom)

# Forward kinematics
vx, vy, omega = forward_kinematics((fl, fr, bl, br), geom)
```

## Usage — C++

```cpp
#include "mecanum_drive_ros2/mecanum_kinematics.hpp"

mecanum_drive::RobotGeometry geom{0.04, 0.215, 0.165};
auto wheels = mecanum_drive::inverseKinematics(0.3, 0.0, 0.0, geom);
auto twist  = mecanum_drive::forwardKinematics(wheels, geom);
```

## Install

```bash
cd ~/ros2_ws/src
ln -s /path/to/Mecanum-Wheel-Robot/packages/mecanum_drive_ros2 .
cd ~/ros2_ws && colcon build --packages-select mecanum_drive_ros2
```

## License

Apache-2.0
