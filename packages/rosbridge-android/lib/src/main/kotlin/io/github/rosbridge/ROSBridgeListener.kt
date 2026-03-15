package io.github.rosbridge

/**
 * Callback interface for ROSBridge WebSocket events.
 *
 * Implement this interface and pass it to [ROSBridgeManager] to receive
 * connection state changes and incoming topic messages.
 */
interface ROSBridgeListener {
    /** Called when the WebSocket connection is successfully established. */
    fun onConnected()

    /** Called when the WebSocket connection is closed normally. */
    fun onDisconnected()

    /** Called when a connection error or unexpected close occurs. */
    fun onError(error: String)

    /**
     * Called for every incoming ROS topic message.
     *
     * @param topic   The ROS topic name (e.g. "/odom").
     * @param message The decoded message payload as a typed map.
     */
    fun onMessageReceived(topic: String, message: Map<String, Any>)
}
