# OmniBot Android App

Native Kotlin MVVM controller app for the OmniBot mecanum-wheel mobile-manipulation robot.
Connects over ROSBridge WebSocket (`ws://robot:9090`) and provides full robot control, 3D
visualisation, real-time mapping, and natural-language AI mission control.

<p align="left">
  <img src="https://img.shields.io/badge/Android-SDK_24+-brightgreen"/>
  <img src="https://img.shields.io/badge/Kotlin-1.9-purple"/>
  <img src="https://img.shields.io/badge/SceneView-2.2.1-blue"/>
  <img src="https://img.shields.io/badge/Hilt-DI-orange"/>
</p>

---

## App Demo

<!-- TODO: Record demo video and replace placeholders with actual GIFs -->

**Dashboard — Live camera, velocity, odometry**

![Dashboard screen](../assets/app_dashboard.gif)
<!-- Placeholder: record gif showing camera feed, BEV overlay toggle, live velocity charts, odometry readout -->

**AI Chat — Natural language robot control**

![AI chat screen](../assets/app_ai_chat.gif)
<!-- Placeholder: record gif typing "go to the kitchen and pick up the red cup", robot executing mission -->

**2D SLAM Map — Real-time occupancy grid**

![SLAM map screen](../assets/app_slam_map.gif)
<!-- Placeholder: record gif showing robot building map in real time, pose overlay updating -->

**3D Point Cloud — Depth camera visualisation**

![Point cloud screen](../assets/app_point_cloud.gif)
<!-- Placeholder: record gif showing live PointCloud2 from Orbbec, pinch-to-zoom, Jet colormap -->

**3D Robot Viewer — Live animated model**

![3D robot viewer screen](../assets/app_robot_viewer.gif)
<!-- Placeholder: record gif showing robot.glb in SceneView, arm joints moving, tap-to-navigate -->

**Controls — Joystick + arm panel**

![Controls screen](../assets/app_controls.gif)
<!-- Placeholder: record gif showing virtual joystick driving robot, arm sliders moving servos -->

---

## Features

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
| Coroutines + StateFlow | ✅ | All live data as StateFlow |

---

## Navigation Structure

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

## Project Structure

```
app/src/main/
├── kotlin/com/varunvaidhiya/robotcontrol/
│   ├── MainActivity.kt
│   ├── data/
│   │   ├── models/                   # OdomData, MapData, JointState, …
│   │   └── repository/
│   │       └── RobotRepository.kt    # Single source of truth, all StateFlows
│   ├── network/
│   │   ├── ROSBridgeManager.kt       # OkHttp WebSocket + ROSBridge v2 JSON
│   │   └── ROSBridgeListener.kt
│   └── ui/
│       ├── common/BaseFragment.kt
│       ├── dashboard/
│       │   ├── DashboardFragment.kt  # Camera, BEV overlay, charts
│       │   └── DashboardViewModel.kt
│       ├── mapping/
│       │   ├── MapFragment.kt        # ViewPager2: 3 tabs
│       │   └── PointCloudFragment.kt + PointCloudViewModel.kt
│       ├── slam/
│       │   ├── SlamMapFragment.kt    # SLAM map + pose overlay
│       │   └── SlamViewModel.kt
│       ├── viewer/
│       │   ├── RobotViewerFragment.kt  # SceneView 3D viewer
│       │   └── RobotViewerViewModel.kt
│       ├── ai/
│       │   ├── AIFragment.kt         # Chat UI + recording toggle
│       │   ├── AIViewModel.kt        # sendCommand(), toggleRecording()
│       │   └── ChatAdapter.kt        # ListAdapter with DiffUtil
│       └── views/
│           ├── PointCloudView.kt     # Canvas + Jet colormap
│           ├── SlamMapView.kt        # OccupancyGrid canvas
│           └── MjpegView.kt          # MJPEG frame decoder
├── res/
│   ├── layout/                       # All XML layouts
│   ├── drawable/                     # Vector icons + backgrounds
│   ├── values/                       # Colors, strings, themes
│   ├── values-night/colors.xml       # Dark mode color overrides
│   └── navigation/mobile_navigation.xml
└── assets/
    └── robot.glb                     # Generated — see 3D Robot Viewer Setup below
```

---

## 3D Robot Viewer Setup

The **3D Robot** tab loads `robot.glb` from `app/src/main/assets/`. This binary is not
checked in. Generate it from the URDF + STL meshes (run from the repo root):

```bash
pip install trimesh[easy] numpy lxml
python tools/urdf_to_glb.py
# Output: android_app/app/src/main/assets/robot.glb
```

Then rebuild the Android app so assets are re-packaged.

The script expands `omnibot.urdf.xacro`, loads all STL meshes, applies joint origin
transforms, and exports a GLB where every node is named after its URDF link
(e.g. `arm_shoulder_pan`). The viewer finds nodes by name and applies live joint
rotations from `/arm/joint_states`.

If `robot.glb` is absent the viewer still shows live pose and joint angles as text
overlays, and displays setup instructions.

---

## Dataset Recording (from the App)

Tap the **REC** button on the Dashboard camera card, or use the toggle in the AI chat
panel. This publishes `true` to `/record_start` and `true` to `/record_stop`.

On the robot side, start the recorder node first:

```bash
ros2 launch omnibot_lerobot teleop_record.launch.py \
    output_dir:=/data/episodes \
    task_name:="pick up the red cube"
```

The node begins writing when it receives the `/record_start` signal.

---

## Build & Run

### Requirements

| Tool | Version |
|------|---------|
| Android Studio | Hedgehog (2023.1.1)+ |
| Minimum SDK | 24 (Android 7.0) |
| Target SDK | 34 (Android 14) |
| Kotlin | 1.9.20+ |
| Gradle | 8.2.0+ |
| JDK | 17 |

### Steps

```bash
# Open android_app/ in Android Studio, let Gradle sync, then:
./gradlew build

# Run on device
./gradlew installDebug
```

### Robot-side prerequisites

```bash
# ROSBridge WebSocket (port 9090)
./launch_rosbridge.sh
# or:
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

# web_video_server for MJPEG (optional but needed for camera feed)
ros2 run web_video_server web_video_server
```

---

## Key Dependencies

| Library | Purpose |
|---------|---------|
| `io.github.sceneview:sceneview:2.2.1` | Filament 3D rendering for robot viewer |
| `com.squareup.okhttp3:okhttp` | WebSocket (ROSBridge) |
| `com.github.PhilJay:MPAndroidChart` | Velocity charts |
| `androidx.viewpager2` | Map tab pager |
| `androidx.recyclerview` | Chat message list |
| `com.google.dagger:hilt-android` | Dependency injection |
| `com.jakewharton.timber:timber` | Logging |

---

## ROS Topics

### Published by the app (Android → robot)

| Topic | Type | Notes |
|-------|------|-------|
| `/cmd_vel` | Twist | 20 Hz, clamped ±1.5 m/s / ±2.0 rad/s |
| `/arm/joint_commands` | JointState | joint names must be `arm_*` prefixed |
| `/arm/enable` | Bool | |
| `/vla/prompt` | String | |
| `/mission/command` | String | natural language or `navigate:pose:x,y,0.0` |
| `/record_start` | Bool | dataset recording start |
| `/record_stop` | Bool | dataset recording stop |
| `/emergency_stop` | Bool | published but not yet consumed robot-side |

### Subscribed by the app (robot → Android)

| Topic | Type |
|-------|------|
| `/odom` | Odometry |
| `/map` | OccupancyGrid |
| `/imu/data` | Imu |
| `/arm/joint_states` | JointState |
| `/camera/depth/points` | PointCloud2 |
| `/mission/status` | String |
| `/diagnostics` | DiagnosticArray |

---

## Future Work

- [ ] **Emergency stop wiring** — Android publishes `/emergency_stop` but no ROS node subscribes yet.
- [ ] **ARCore mode** — AR overlay of robot model on live camera using ARCore + SceneView AR session.
- [ ] **Voice input** — Android SpeechRecognizer feeding the AI chat interface.
- [ ] **Multi-camera switcher** — Swipe between wrist / front / BEV camera on Dashboard.
- [ ] **Foxglove debug panel** — Embedded Foxglove WebView for raw topic inspection.
- [ ] **Unit tests for RobotRepository** — Refactor singleton to constructor DI first.
- [ ] **CI GLB generation** — GitHub Actions step to run `tools/urdf_to_glb.py` and upload `robot.glb` as a release asset.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Gradle sync failed | Check internet; **File > Invalidate Caches > Restart** |
| Build errors | Ensure JDK 17; **Build > Clean Project** |
| WebSocket not connecting | Verify same network; check `./launch_rosbridge.sh` is running |
| No camera image | Confirm `web_video_server` is running; check camera URL in Settings |
| 3D viewer shows banner | Run `python tools/urdf_to_glb.py` then rebuild app |
| Robot model doesn't move | Verify `/odom` and `/arm/joint_states` are publishing |

---

## License

MIT

## Contact

Varun Vaidhiya — varun.vaidhiya@gmail.com
