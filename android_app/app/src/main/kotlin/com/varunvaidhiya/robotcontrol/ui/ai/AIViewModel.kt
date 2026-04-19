package com.varunvaidhiya.robotcontrol.ui.ai

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.varunvaidhiya.robotcontrol.data.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import javax.inject.Inject

data class ChatMessage(
    val text: String,
    val isUser: Boolean,
    val timestamp: String = SimpleDateFormat("HH:mm", Locale.getDefault()).format(Date())
)

@HiltViewModel
class AIViewModel @Inject constructor(
    private val repository: RobotRepository
) : ViewModel() {

    val connectionState = repository.connectionState
    val isRecording     = repository.isRecording

    private val _messages = MutableStateFlow<List<ChatMessage>>(
        listOf(ChatMessage("Hi Varun! I'm ready. Tell me what to do — navigate somewhere, pick an object, or explore the area.", isUser = false))
    )
    val messages: StateFlow<List<ChatMessage>> = _messages.asStateFlow()

    init {
        // Listen for mission status updates from the robot
        viewModelScope.launch {
            repository.missionStatus.collect { status ->
                if (status.isNotBlank()) {
                    appendMessage(ChatMessage(status, isUser = false))
                }
            }
        }
    }

    fun sendCommand(text: String) {
        if (text.isBlank()) return

        appendMessage(ChatMessage(text, isUser = true))

        // Route to ROS mission planner — supports "navigate:X,vla:Y" or plain prompt
        val lower = text.lowercase(Locale.getDefault())
        val rosCommand = when {
            lower.startsWith("navigate") || lower.startsWith("go to") ->
                "navigate:${text.substringAfter(" ")}"
            lower.startsWith("vla:") || lower.startsWith("pick") || lower.startsWith("grab") || lower.startsWith("find") ->
                "vla:$text"
            else ->
                text  // raw mission command
        }

        repository.sendMissionCommand(rosCommand)

        // Immediate acknowledgement
        appendMessage(ChatMessage("Got it! Executing: \"$rosCommand\"", isUser = false))
    }

    fun toggleRecording(enable: Boolean) {
        if (enable) {
            val bagName = "omnibot_${System.currentTimeMillis()}"
            repository.startRecording(bagName)
            appendMessage(ChatMessage("Started dataset recording → $bagName.bag", isUser = false))
        } else {
            repository.stopRecording()
            appendMessage(ChatMessage("Stopped recording. Bag saved.", isUser = false))
        }
    }

    private fun appendMessage(msg: ChatMessage) {
        _messages.value = _messages.value + msg
    }
}
