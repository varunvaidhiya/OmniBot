package io.github.rosbridge

import com.google.gson.Gson
import kotlinx.coroutines.*
import okhttp3.*
import java.util.concurrent.TimeUnit

/**
 * ROSBridge WebSocket client for Android.
 *
 * Manages the WebSocket connection to a [rosbridge_server](https://github.com/RobotWebTools/rosbridge_suite)
 * instance, with automatic exponential-backoff reconnection and a keep-alive heartbeat.
 *
 * Usage:
 * ```kotlin
 * val manager = ROSBridgeManager("ws://192.168.1.100:9090", listener)
 * manager.connect()
 * manager.subscribe("/odom", "nav_msgs/Odometry")
 * manager.publish("/cmd_vel", mapOf("linear" to mapOf("x" to 0.2, "y" to 0.0, "z" to 0.0),
 *                                   "angular" to mapOf("x" to 0.0, "y" to 0.0, "z" to 0.0)))
 * // later:
 * manager.disconnect()
 * ```
 *
 * @param serverUrl  WebSocket URL of the rosbridge server (e.g. "ws://robot.local:9090").
 * @param listener   [ROSBridgeListener] to receive events and messages.
 */
class ROSBridgeManager(
    private val serverUrl: String,
    private val listener: ROSBridgeListener,
) {
    private val client = OkHttpClient.Builder()
        .readTimeout(0, TimeUnit.MILLISECONDS)
        .build()

    private var webSocket: WebSocket? = null
    private val gson = Gson()
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())

    private var reconnectAttempts = 0
    private val maxReconnectAttempts = 5
    private val baseReconnectDelayMs = 1_000L

    /** Current WebSocket connection state. */
    var connectionState: ConnectionState = ConnectionState.DISCONNECTED
        private set

    enum class ConnectionState {
        DISCONNECTED, CONNECTING, CONNECTED, ERROR
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Open the WebSocket connection. No-op if already connected or connecting. */
    fun connect() {
        if (connectionState == ConnectionState.CONNECTED ||
            connectionState == ConnectionState.CONNECTING) return

        connectionState = ConnectionState.CONNECTING
        val request = Request.Builder().url(serverUrl).build()
        webSocket = client.newWebSocket(request, createListener())
    }

    /** Close the WebSocket connection cleanly. */
    fun disconnect() {
        webSocket?.close(1000, "Client disconnecting")
        webSocket = null
        connectionState = ConnectionState.DISCONNECTED
        scope.coroutineContext.cancelChildren()
    }

    /**
     * Subscribe to a ROS topic.
     *
     * @param topic       ROS topic name (e.g. "/odom").
     * @param messageType ROS message type (e.g. "nav_msgs/Odometry").
     */
    fun subscribe(topic: String, messageType: String) {
        send(mapOf("op" to "subscribe", "topic" to topic, "type" to messageType))
    }

    /**
     * Unsubscribe from a ROS topic.
     *
     * @param topic ROS topic name.
     */
    fun unsubscribe(topic: String) {
        send(mapOf("op" to "unsubscribe", "topic" to topic))
    }

    /**
     * Publish a message to a ROS topic.
     *
     * @param topic   ROS topic name.
     * @param message Message payload as a Kotlin Map mirroring the ROS message fields.
     */
    fun publish(topic: String, message: Map<String, Any>) {
        send(mapOf("op" to "publish", "topic" to topic, "msg" to message))
    }

    /**
     * Call a ROS service.
     *
     * @param service   Service name (e.g. "/clear").
     * @param args      Service arguments map.
     * @param id        Optional call ID for correlating responses.
     */
    fun callService(service: String, args: Map<String, Any> = emptyMap(), id: String? = null) {
        val msg = mutableMapOf<String, Any>("op" to "call_service", "service" to service, "args" to args)
        if (id != null) msg["id"] = id
        send(msg)
    }

    // ── Internal ──────────────────────────────────────────────────────────────

    private fun send(message: Map<String, Any>) {
        if (connectionState != ConnectionState.CONNECTED) return
        webSocket?.send(gson.toJson(message))
    }

    private fun attemptReconnect() {
        if (reconnectAttempts >= maxReconnectAttempts) {
            connectionState = ConnectionState.ERROR
            return
        }
        val delayMs = baseReconnectDelayMs * (1L shl reconnectAttempts)
        reconnectAttempts++
        scope.launch {
            delay(delayMs)
            connect()
        }
    }

    private fun startHeartbeat() {
        scope.launch {
            while (isActive && connectionState == ConnectionState.CONNECTED) {
                delay(5_000L)
                webSocket?.send("{}")  // empty JSON ping
            }
        }
    }

    private fun createListener() = object : WebSocketListener() {
        override fun onOpen(webSocket: WebSocket, response: Response) {
            connectionState = ConnectionState.CONNECTED
            reconnectAttempts = 0
            listener.onConnected()
            startHeartbeat()
        }

        override fun onMessage(webSocket: WebSocket, text: String) {
            try {
                @Suppress("UNCHECKED_CAST")
                val msg = gson.fromJson(text, Map::class.java) as Map<String, Any>
                val topic = msg["topic"] as? String ?: return
                @Suppress("UNCHECKED_CAST")
                val payload = msg["msg"] as? Map<String, Any> ?: return
                listener.onMessageReceived(topic, payload)
            } catch (_: Exception) { /* malformed JSON — ignore */ }
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            connectionState = ConnectionState.ERROR
            listener.onError(t.message ?: "Unknown error")
            attemptReconnect()
        }

        override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
            connectionState = ConnectionState.DISCONNECTED
            listener.onDisconnected()
        }
    }
}
