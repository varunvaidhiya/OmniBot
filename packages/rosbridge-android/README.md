# rosbridge-android

> Android client library for the [ROSBridge WebSocket protocol](https://github.com/RobotWebTools/rosbridge_suite).
> The missing Kotlin library for building ROS robot control apps.

## Features

- **`ROSBridgeManager`** — Kotlin coroutines WebSocket client with:
  - subscribe / unsubscribe / publish / call_service
  - Exponential-backoff auto-reconnect (5 attempts: 1 s, 2 s, 4 s, 8 s, 16 s)
  - 5-second keep-alive heartbeat
- **`VirtualJoystickView`** — Custom `View` with configurable deadzone and clamping,
  outputs normalized (x, y) in [-1, 1]
- **`SlamMapView`** — Renders ROS OccupancyGrid with robot pose overlay,
  supports pinch-zoom and drag-pan

## Install via JitPack

```gradle
// settings.gradle
dependencyResolutionManagement {
    repositories {
        maven { url 'https://jitpack.io' }
    }
}

// app/build.gradle
dependencies {
    implementation 'com.github.varunvaidhiya.Mecanum-Wheel-Robot:rosbridge-android:1.0.0'
}
```

## Usage

### Connect to ROSBridge

```kotlin
val manager = ROSBridgeManager(
    serverUrl = "ws://192.168.1.100:9090",
    listener  = object : ROSBridgeListener {
        override fun onConnected()          { Log.d("ROS", "Connected") }
        override fun onDisconnected()       { Log.d("ROS", "Disconnected") }
        override fun onError(error: String) { Log.e("ROS", error) }
        override fun onMessageReceived(topic: String, message: Map<String, Any>) {
            when (topic) {
                "/odom" -> handleOdom(message)
                "/map"  -> handleMap(message)
            }
        }
    }
)

manager.connect()

// Subscribe to topics
manager.subscribe("/odom", "nav_msgs/Odometry")
manager.subscribe("/map",  "nav_msgs/OccupancyGrid")

// Publish cmd_vel
manager.publish("/cmd_vel", mapOf(
    "linear"  to mapOf("x" to 0.2, "y" to 0.0, "z" to 0.0),
    "angular" to mapOf("x" to 0.0, "y" to 0.0, "z" to 0.0),
))

// Cleanup
manager.disconnect()
```

### Virtual Joystick

```xml
<!-- layout.xml -->
<io.github.rosbridge.ui.VirtualJoystickView
    android:id="@+id/joystick"
    android:layout_width="200dp"
    android:layout_height="200dp" />
```

```kotlin
joystick.onJoystickMoved = { x, y ->
    // y = forward/backward, x = strafe (ROS REP 103 convention)
    manager.publish("/cmd_vel", mapOf(
        "linear"  to mapOf("x" to y, "y" to -x, "z" to 0.0),
        "angular" to mapOf("x" to 0.0, "y" to 0.0, "z" to 0.0),
    ))
}
```

### SLAM Map View

```xml
<io.github.rosbridge.ui.SlamMapView
    android:id="@+id/slamMap"
    android:layout_width="match_parent"
    android:layout_height="300dp" />
```

```kotlin
// On /map message
slamMapView.updateMap(
    width  = (info["width"]  as Double).toInt(),
    height = (info["height"] as Double).toInt(),
    data   = (msg["data"] as List<*>).map { (it as Double).toInt() }.toIntArray(),
)

// On /amcl_pose message
slamMapView.updateRobotPose(poseX, poseY, yaw)
```

## Requirements

- Android SDK 24+
- Kotlin 1.9+
- OkHttp 4.x (transitively included)

## License

Apache-2.0
