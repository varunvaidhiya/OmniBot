package com.varunvaidhiya.robotcontrol.data.repository

import com.varunvaidhiya.robotcontrol.data.models.*
import com.varunvaidhiya.robotcontrol.network.ROSBridgeListener
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager
import com.varunvaidhiya.robotcontrol.utils.Constants
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import timber.log.Timber
import kotlin.math.atan2
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Single source of truth for robot data.
 * Manages ROSBridge connection and exposes data as StateFlows.
 */
@Singleton
class RobotRepository @Inject constructor() {

    private var rosManager: ROSBridgeManager? = null

    // ── Connection ────────────────────────────────────────────────────────────
    private val _connectionState = MutableStateFlow(ROSBridgeManager.ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ROSBridgeManager.ConnectionState> = _connectionState.asStateFlow()

    // ── Odometry (/odom) ──────────────────────────────────────────────────────
    private val _odomData = MutableStateFlow(OdomData())
    val odomData: StateFlow<OdomData> = _odomData.asStateFlow()

    // ── SLAM Map (/map) ───────────────────────────────────────────────────────
    private val _mapData = MutableStateFlow(MapData())
    val mapData: StateFlow<MapData> = _mapData.asStateFlow()

    // ── Robot Status (/robot_status, legacy) ──────────────────────────────────
    private val _robotStatus = MutableStateFlow(RobotStatus())
    val robotStatus: StateFlow<RobotStatus> = _robotStatus.asStateFlow()

    // ── Motor / Wheel data (/motor_pwm, /wheel_speeds) ────────────────────────
    private val _motorData = MutableStateFlow(
        MotorData(MotorStatus(), MotorStatus(), MotorStatus(), MotorStatus())
    )
    val motorData: StateFlow<MotorData> = _motorData.asStateFlow()

    private val _wheelSpeeds = MutableStateFlow(WheelSpeed())
    val wheelSpeeds: StateFlow<WheelSpeed> = _wheelSpeeds.asStateFlow()

    // ── Arm joints (/arm/joint_states) ────────────────────────────────────────
    // Ordered by Constants.ARM_JOINT_NAMES: shoulder_pan, shoulder_lift,
    // elbow_flex, wrist_flex, wrist_roll, gripper (radians).
    private val _armJointPositions = MutableStateFlow(
        DoubleArray(Constants.ARM_JOINT_NAMES.size) { 0.0 }
    )
    val armJointPositions: StateFlow<DoubleArray> = _armJointPositions.asStateFlow()

    // ── ROSBridge listener ────────────────────────────────────────────────────
    private val rosListener = object : ROSBridgeListener {
        override fun onConnected() {
            _connectionState.value = ROSBridgeManager.ConnectionState.CONNECTED
            _robotStatus.value = _robotStatus.value.copy(isConnected = true)
            subscribeToTopics()
        }

        override fun onDisconnected() {
            _connectionState.value = ROSBridgeManager.ConnectionState.DISCONNECTED
            _robotStatus.value = _robotStatus.value.copy(isConnected = false)
        }

        override fun onError(error: String) {
            _connectionState.value = ROSBridgeManager.ConnectionState.ERROR
            Timber.e("ROS Error: $error")
        }

        override fun onMessageReceived(topic: String, message: Map<String, Any>) {
            handleMessage(topic, message)
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    fun connect(ip: String = Constants.DEFAULT_ROBOT_IP, port: Int = Constants.DEFAULT_ROSBRIDGE_PORT) {
        _robotStatus.value = _robotStatus.value.copy(ipAddress = ip)
        val url = "ws://$ip:$port"
        rosManager = ROSBridgeManager(url, rosListener)
        rosManager?.connect()
        _connectionState.value = ROSBridgeManager.ConnectionState.CONNECTING
    }

    fun disconnect() {
        rosManager?.disconnect()
    }

    fun sendVelocity(command: VelocityCommand) {
        rosManager?.publish(Constants.TOPIC_CMD_VEL, command.toROSTwist())
    }

    fun sendEmergencyStop() {
        rosManager?.publish(Constants.TOPIC_EMERGENCY_STOP, mapOf("data" to true))
    }

    /**
     * Send arm joint positions in radians.
     * [positions] must be aligned with Constants.ARM_JOINT_NAMES.
     * Publishes sensor_msgs/JointState to /arm/joint_commands.
     */
    fun sendArmJointCommand(positions: DoubleArray) {
        val names = Constants.ARM_JOINT_NAMES
        val posList = positions.toList()
        rosManager?.publish(
            Constants.TOPIC_ARM_JOINT_COMMANDS,
            mapOf(
                "name"     to names,
                "position" to posList,
                "velocity" to emptyList<Double>(),
                "effort"   to emptyList<Double>()
            )
        )
    }

    /** Enable or disable arm torque (std_msgs/Bool → /arm/enable). */
    fun setArmEnabled(enabled: Boolean) {
        rosManager?.publish(Constants.TOPIC_ARM_ENABLE, mapOf("data" to enabled))
    }

    fun sendMode(mode: RobotMode) {
        rosManager?.publish(Constants.TOPIC_ROBOT_MODE, mapOf("data" to mode.name))
        _robotStatus.value = _robotStatus.value.copy(currentMode = mode)
    }

    // ── Subscriptions ─────────────────────────────────────────────────────────

    private fun subscribeToTopics() {
        rosManager?.apply {
            // Standard ROS topics
            subscribe(Constants.TOPIC_ODOM,             "nav_msgs/Odometry")
            subscribe(Constants.TOPIC_MAP,               "nav_msgs/OccupancyGrid")
            subscribe(Constants.TOPIC_IMU,               "sensor_msgs/Imu")
            subscribe(Constants.TOPIC_DIAGNOSTICS,       "diagnostic_msgs/DiagnosticArray")
            // Arm joint feedback
            subscribe(Constants.TOPIC_ARM_JOINT_STATES, "sensor_msgs/JointState")
            // Legacy custom topics
            subscribe(Constants.TOPIC_ROBOT_STATUS,     "robot_msgs/RobotStatus")
            subscribe(Constants.TOPIC_WHEEL_SPEEDS,     "robot_msgs/WheelSpeed")
            subscribe(Constants.TOPIC_MOTOR_PWM,        "robot_msgs/MotorData")
        }
    }

    private fun handleMessage(topic: String, message: Map<String, Any>) {
        try {
            when (topic) {
                Constants.TOPIC_ODOM             -> parseOdom(message)
                Constants.TOPIC_MAP              -> parseMap(message)
                Constants.TOPIC_ARM_JOINT_STATES -> parseArmJointStates(message)
                Constants.TOPIC_ROBOT_STATUS     -> parseRobotStatus(message)
                Constants.TOPIC_WHEEL_SPEEDS     -> parseWheelSpeeds(message)
                Constants.TOPIC_MOTOR_PWM        -> parseMotorData(message)
            }
        } catch (e: Exception) {
            Timber.e(e, "Error parsing message for topic: $topic")
        }
    }

    // ── Parsers ───────────────────────────────────────────────────────────────

    @Suppress("UNCHECKED_CAST")
    private fun parseOdom(msg: Map<String, Any>) {
        val poseWrapper  = msg["pose"]   as? Map<String, Any> ?: return
        val pose         = poseWrapper["pose"] as? Map<String, Any> ?: return
        val position     = pose["position"]    as? Map<String, Any> ?: return
        val orientation  = pose["orientation"] as? Map<String, Any> ?: return

        val twistWrapper = msg["twist"]  as? Map<String, Any> ?: return
        val twist        = twistWrapper["twist"] as? Map<String, Any> ?: return
        val linear       = twist["linear"]  as? Map<String, Any> ?: return
        val angular      = twist["angular"] as? Map<String, Any> ?: return

        val x = (position["x"] as? Number)?.toFloat() ?: 0f
        val y = (position["y"] as? Number)?.toFloat() ?: 0f
        val qx = (orientation["x"] as? Number)?.toDouble() ?: 0.0
        val qy = (orientation["y"] as? Number)?.toDouble() ?: 0.0
        val qz = (orientation["z"] as? Number)?.toDouble() ?: 0.0
        val qw = (orientation["w"] as? Number)?.toDouble() ?: 1.0
        val yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)).toFloat()

        val vx = (linear["x"]  as? Number)?.toFloat() ?: 0f
        val vy = (linear["y"]  as? Number)?.toFloat() ?: 0f
        val vz = (angular["z"] as? Number)?.toFloat() ?: 0f

        _odomData.value = OdomData(x = x, y = y, yaw = yaw, vx = vx, vy = vy, vz = vz)
    }

    @Suppress("UNCHECKED_CAST")
    private fun parseMap(msg: Map<String, Any>) {
        val info   = msg["info"] as? Map<String, Any> ?: return
        val width  = (info["width"]  as? Number)?.toInt()   ?: return
        val height = (info["height"] as? Number)?.toInt()   ?: return
        val res    = (info["resolution"] as? Number)?.toFloat() ?: 0.05f

        val origin   = info["origin"]   as? Map<String, Any>
        val originPos = origin?.get("position") as? Map<String, Any>
        val originX  = (originPos?.get("x") as? Number)?.toFloat() ?: 0f
        val originY  = (originPos?.get("y") as? Number)?.toFloat() ?: 0f

        @Suppress("UNCHECKED_CAST")
        val rawData = msg["data"] as? List<Number> ?: return
        val cells = IntArray(rawData.size) { rawData[it].toInt() }

        _mapData.value = MapData(
            width = width, height = height,
            resolution = res,
            originX = originX, originY = originY,
            cells = cells
        )
    }

    private fun parseRobotStatus(msg: Map<String, Any>) {
        val isConnected = msg["connected"] as? Boolean ?: true
        _robotStatus.value = _robotStatus.value.copy(
            isConnected = isConnected,
            timestamp = System.currentTimeMillis()
        )
    }

    private fun parseWheelSpeeds(msg: Map<String, Any>) {
        val fl = (msg["front_left"]  as? Number)?.toFloat() ?: 0f
        val fr = (msg["front_right"] as? Number)?.toFloat() ?: 0f
        val bl = (msg["back_left"]   as? Number)?.toFloat() ?: 0f
        val br = (msg["back_right"]  as? Number)?.toFloat() ?: 0f
        _wheelSpeeds.value = WheelSpeed(frontLeft = fl, frontRight = fr, backLeft = bl, backRight = br)
    }

    @Suppress("UNCHECKED_CAST")
    private fun parseArmJointStates(msg: Map<String, Any>) {
        val names     = (msg["name"]     as? List<*>)?.filterIsInstance<String>() ?: return
        val positions = (msg["position"] as? List<*>)?.map { (it as? Number)?.toDouble() ?: 0.0 } ?: return
        val nameToPos = names.zip(positions).toMap()
        val updated = DoubleArray(Constants.ARM_JOINT_NAMES.size) { i ->
            nameToPos[Constants.ARM_JOINT_NAMES[i]] ?: _armJointPositions.value[i]
        }
        _armJointPositions.value = updated
    }

    private fun parseMotorData(msg: Map<String, Any>) {
        fun motorStatus(key: String): MotorStatus {
            @Suppress("UNCHECKED_CAST")
            val m = msg[key] as? Map<String, Any> ?: return MotorStatus()
            return MotorStatus(
                pwmValue    = (m["pwm"]         as? Number)?.toInt()   ?: 0,
                rpm         = (m["rpm"]         as? Number)?.toFloat() ?: 0f,
                current     = (m["current"]     as? Number)?.toFloat() ?: 0f,
                temperature = (m["temperature"] as? Number)?.toFloat() ?: 0f
            )
        }
        _motorData.value = MotorData(
            frontLeft  = motorStatus("front_left"),
            frontRight = motorStatus("front_right"),
            backLeft   = motorStatus("back_left"),
            backRight  = motorStatus("back_right")
        )
    }
}
