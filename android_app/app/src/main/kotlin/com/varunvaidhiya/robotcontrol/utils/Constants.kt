package com.varunvaidhiya.robotcontrol.utils

/**
 * Application-wide constants
 */
object Constants {
    // Network
    const val DEFAULT_ROBOT_IP = "192.168.1.100"
    const val DEFAULT_ROSBRIDGE_PORT = 9090
    const val CONNECTION_TIMEOUT_MS = 10000L
    
    // Control
    const val JOYSTICK_DEADZONE = 0.1f
    const val MAX_LINEAR_VELOCITY = 1.5f  // m/s
    const val MAX_ANGULAR_VELOCITY = 2.0f // rad/s
    const val CMD_VEL_PUBLISH_RATE_HZ = 20
    
    // ROS Topics — publish
    const val TOPIC_CMD_VEL = "/cmd_vel"
    const val TOPIC_EMERGENCY_STOP = "/emergency_stop"
    const val TOPIC_ROBOT_MODE = "/robot_mode"

    // ROS Topics — subscribe
    const val TOPIC_ODOM = "/odom"
    const val TOPIC_MAP = "/map"
    const val TOPIC_IMU = "/imu/data"
    const val TOPIC_DIAGNOSTICS = "/diagnostics"

    // Arm topics — SO-101 6-DOF manipulator
    const val TOPIC_ARM_JOINT_COMMANDS = "/arm/joint_commands"  // publish (sensor_msgs/JointState)
    const val TOPIC_ARM_JOINT_STATES   = "/arm/joint_states"    // subscribe (sensor_msgs/JointState)
    const val TOPIC_ARM_ENABLE         = "/arm/enable"          // publish (std_msgs/Bool)

    // URDF joint names for the arm (must match omnibot.urdf.xacro exactly)
    val ARM_JOINT_NAMES = listOf(
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper"
    )

    // Legacy custom topics (kept for compatibility)
    const val TOPIC_ROBOT_STATUS = "/robot_status"
    const val TOPIC_WHEEL_SPEEDS = "/wheel_speeds"
    const val TOPIC_MOTOR_PWM = "/motor_pwm"
    
    // UI
    const val CHART_MAX_DATA_POINTS = 300
    const val DASHBOARD_UPDATE_RATE_HZ = 10
    
    // Logging
    const val LOG_FILE_MAX_SIZE_MB = 50
    const val LOG_RETENTION_DAYS = 7
}
