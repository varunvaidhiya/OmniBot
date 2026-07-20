# Contributing to OmniBot Open

Thanks for your interest in OmniBot Open! This is a community hardware +
teleoperation project and contributions of all sizes are welcome — from a
typo fix to a whole new teleoperation backend.

## Where help is most wanted

| Area | What's needed |
|---|---|
| **Hardware docs** | Assembly photos, CAD/STEP files, BOM links and up-to-date prices |
| **VR teleoperation** | Implement `robot_ws/src/omnibot_vr_teleop` (see its README) |
| **New teleop methods** | Phone-browser gamepad, web UI, SpaceMouse, keyboard |
| **Camera calibration** | Intrinsics + extrinsics presets for the BEV stitcher |
| **Tests** | Driver unit tests, kinematics edge cases, protocol fuzzing |

Please open an issue to discuss anything larger than a bug fix **before**
starting, so we can avoid duplicate work.

## Development setup

```bash
git clone https://github.com/varunvaidhiya/OmniBot-Open
cd OmniBot-Open

# ROS 2 workspace
source /opt/ros/jazzy/setup.bash
cd robot_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

The pure-Python libraries (`packages/yahboom_ros2`, `packages/mecanum_drive_ros2`)
have **no ROS dependency** and can be developed and tested with plain `pytest`.

## Running the tests

```bash
# Inside the built workspace:
cd robot_ws
colcon test --packages-select omnibot_arm omnibot_driver
colcon test-result --verbose

# Or the hardware-free libraries standalone:
cd packages/mecanum_drive_ros2 && python -m pytest tests/
cd packages/yahboom_ros2       && python -m pytest tests/
```

Every node that touches hardware (`arm_driver_node`, `configure_servos`,
`yahboom_driver`) has a **simulation fallback** so it can be exercised
without a robot attached. Please keep that property when adding features:
guard hardware imports and degrade gracefully.

## Coding conventions

- **Python**: PEP 8, type hints on public functions, module-level docstrings
  that state topics published/subscribed for ROS nodes.
- **C++**: C++17, `-Wall -Wextra -Wpedantic` clean.
- **Kotlin** (Android app): MVVM, one ViewModel per screen, no logic in
  Fragments.
- Match the surrounding file's style. Keep hardware-facing constants in
  `config/*.yaml`, not hard-coded in nodes.

## Pull requests

1. Branch from `main`.
2. Keep the change focused; unrelated cleanups belong in their own PR.
3. Add or update tests for any logic change.
4. Make sure `colcon test` (or `pytest` for the standalone libraries) passes.
5. Update the relevant README / docs when you change behaviour or add params.
6. Open the PR as a draft until CI is green, then mark it ready for review.

## Reporting bugs

Open an issue with: what you expected, what happened, the exact command you
ran, your ROS distro, and (for hardware issues) the board/servo model and
`dmesg | grep tty` output so we can see how the device enumerated.

## License

By contributing you agree that your contributions are licensed under the
project's [Apache 2.0](LICENSE) license.
