# OmniBot Open — Bill of Materials

A reference BOM for reproducing the OmniBot Open platform: a mecanum-wheel
mobile base carrying a 6-DOF SO-101 arm, driven by a Raspberry Pi 5 running
ROS 2 Jazzy.

> Prices are **approximate** (USD, early 2026) and vary by region and
> supplier. Treat this as a shopping checklist, not a fixed quote. If you
> find better sources, a PR updating this table is very welcome (see
> [CONTRIBUTING.md](../CONTRIBUTING.md)).

## Core compute & drive

| # | Component | Qty | ~Unit | Notes |
|---|---|---:|---:|---|
| 1 | Raspberry Pi 5 (8 GB) | 1 | $80 | Robot brain — runs all ROS 2 nodes |
| 2 | Active cooler / heatsink for Pi 5 | 1 | $5 | The Pi 5 throttles under sustained load |
| 3 | microSD card (64 GB, A2) | 1 | $10 | Ubuntu 24.04 + ROS 2 Jazzy |
| 4 | Yahboom ROS Robot Expansion Board (Rosmaster X3) | 1 | $70 | Mecanum motor driver + IMU, USB serial |
| 5 | Mecanum wheels, 40 mm radius (2× left, 2× right) | 4 | $12 | Handedness matters — order 2 of each |
| 6 | TT / encoder DC gear motors (matched to board) | 4 | $8 | Usually bundled with the Yahboom kit |

## Manipulator

| # | Component | Qty | ~Unit | Notes |
|---|---|---:|---:|---|
| 7 | SO-101 6-DOF arm kit (LeRobot-compatible) | 1 | $150 | Follower arm |
| 8 | Feetech STS3215 serial bus servos | 7 | $15 | 6 joints + 1 spare; leader arm needs its own set |
| 9 | Feetech serial bus driver board (URT / TTL) | 1 | $10 | USB ↔ servo bus, enumerates as `/dev/ttyACM*` |
| 10 | SO-101 leader arm (for teleop data collection) | 1 | $150 | Optional — only for leader-follower recording |

## Perception

| # | Component | Qty | ~Unit | Notes |
|---|---|---:|---:|---|
| 11 | USB cameras (base-mounted, wide FOV) | 4 | $12 | Feed the BEV stitcher |
| 12 | USB camera (wrist-mounted) | 1 | $12 | Close-up manipulation view |
| 13 | Orbbec Astra Pro RGB-D camera | 1 | $150 | **Optional** — depth + 3D point cloud |
| 14 | Powered USB 3.0 hub (7-port) | 1 | $25 | 5 cameras + 2 servo buses exceed the Pi's ports |

## Teleoperation & power

| # | Component | Qty | ~Unit | Notes |
|---|---|---:|---:|---|
| 15 | Xbox wireless controller + USB dongle | 1 | $50 | Base + record control |
| 16 | 12 V LiPo / Li-ion battery pack (3S, ≥5 Ah) | 1 | $35 | Drives motors + board |
| 17 | 12 V → 5 V/5 A buck converter (USB-C) | 1 | $12 | Clean 5 V for the Pi 5 |
| 18 | Inline fuse + main power switch | 1 | $8 | **Do not skip** — see the safety note below |
| 19 | Emergency-stop button (latching, NC) | 1 | $10 | Cuts motor power independently of software |

## Structure & fasteners

| # | Component | Qty | ~Unit | Notes |
|---|---|---:|---:|---|
| 20 | Chassis plate / 3D-printed base | 1 | — | See the URDF for footprint dimensions |
| 21 | M2.5 / M3 standoffs, screws, nuts (assortment) | 1 | $15 | Pi, board, camera mounts |
| 22 | Thread-locker (medium, removable) | 1 | $6 | On every motor-shaft and arm-base fastener |
| 23 | Cable sleeving, zip ties, spiral wrap | 1 | $10 | Keep leads away from the wheels and joints |

## Approximate totals

| Configuration | Rough cost |
|---|---|
| **Base + arm, teleop only** (no depth cam, no leader arm) | ~$620 |
| **Full data-collection rig** (leader arm + Astra + 5 cams) | ~$960 |

## Reference dimensions

The URDF is the source of truth for geometry. Key values used by the
kinematics and drivers:

| Parameter | Value | Source |
|---|---|---|
| Wheel radius | 0.04 m | `packages/yahboom_ros2/config/yahboom_params.yaml` |
| Track width (left–right) | 0.215 m | same |
| Wheelbase (front–rear) | 0.165 m | same |
| Arm servo resolution | 4096 ticks / rev | `omnibot_arm/config/arm_params.yaml` |

See `robot_ws/src/omnibot_description/urdf/omnibot.urdf.xacro` for full link
and joint placement.

> ⚠️ **Power safety.** Mecanum motors stall hard when the chassis is
> obstructed and can pull large currents. Always wire the inline fuse (item
> 18) and a latching emergency stop (item 19) that cuts **motor** power
> directly — never rely on software alone to stop the wheels.
