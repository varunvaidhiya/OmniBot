package com.varunvaidhiya.robotcontrol.ui.controls

import androidx.lifecycle.ViewModel
import com.varunvaidhiya.robotcontrol.data.models.RobotMode
import com.varunvaidhiya.robotcontrol.data.models.VelocityCommand
import com.varunvaidhiya.robotcontrol.data.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class ControlsViewModel @Inject constructor(
    private val repository: RobotRepository
) : ViewModel() {

    val robotStatus = repository.robotStatus

    fun sendVelocity(linearX: Float, linearY: Float, angular: Float) {
        repository.sendVelocity(VelocityCommand(linearX, linearY, angular))
    }

    fun triggerEmergencyStop() {
        repository.sendEmergencyStop()
    }

    fun setMode(mode: RobotMode) {
        repository.sendMode(mode)
    }
}
