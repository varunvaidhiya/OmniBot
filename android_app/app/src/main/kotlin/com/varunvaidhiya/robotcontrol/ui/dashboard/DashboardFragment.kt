package com.varunvaidhiya.robotcontrol.ui.dashboard

import android.graphics.Color
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.varunvaidhiya.robotcontrol.R
import com.varunvaidhiya.robotcontrol.data.models.OdomData
import com.varunvaidhiya.robotcontrol.databinding.FragmentDashboardBinding
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager
import com.varunvaidhiya.robotcontrol.ui.camera.CameraViewModel
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import com.varunvaidhiya.robotcontrol.utils.Constants
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

@AndroidEntryPoint
class DashboardFragment : BaseFragment<FragmentDashboardBinding>() {

    private val dashboardViewModel: DashboardViewModel by viewModels()
    private val cameraViewModel: CameraViewModel by viewModels()

    private var chartEntryIndex = 0f

    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentDashboardBinding {
        return FragmentDashboardBinding.inflate(inflater, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        setupObservers()
    }

    private fun setupObservers() {
        // Observe Connection State
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                dashboardViewModel.connectionState.collectLatest { state ->
                    updateConnectionStatus(state)
                }
            }
        }
        
        // Observe Robot Status (IP)
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                dashboardViewModel.robotStatus.collectLatest { status ->
                    binding.textIpAddress.text = "IP: ${if (status.ipAddress.isEmpty()) "--" else status.ipAddress}"
                }
            }
        }
        
        // Observe Camera URL
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                cameraViewModel.streamUrl.collectLatest { url ->
                    if (url.isNotEmpty()) {
                        binding.mjpegView.startStream(url)
                    } else {
                        binding.mjpegView.stopStream()
                    }
                }
            }
        }
        
        // Setup Chart
        setupChart()

        // Observe Odometry — feed live vx/vy/vz into velocity chart
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                dashboardViewModel.odomData.collectLatest { odom ->
                    updateChart(odom)
                }
            }
        }
    }
    
    private fun setupChart() {
        val setVx = LineDataSet(ArrayList(), "Vx (m/s)").apply {
            color = Color.parseColor("#1976D2")   // blue
            setDrawCircles(false); setDrawValues(false); lineWidth = 1.5f
        }
        val setVy = LineDataSet(ArrayList(), "Vy (m/s)").apply {
            color = Color.parseColor("#388E3C")   // green
            setDrawCircles(false); setDrawValues(false); lineWidth = 1.5f
        }
        val setVz = LineDataSet(ArrayList(), "Vz (rad/s)").apply {
            color = Color.parseColor("#F57C00")   // orange
            setDrawCircles(false); setDrawValues(false); lineWidth = 1.5f
        }

        binding.chartVelocity.apply {
            data = LineData(setVx, setVy, setVz)
            description.isEnabled = false
            setTouchEnabled(false)
            setDrawGridBackground(false)
            axisRight.isEnabled = false
            xAxis.setDrawLabels(false)
            legend.isEnabled = true
            setVisibleXRangeMaximum(Constants.CHART_MAX_DATA_POINTS.toFloat())
        }
    }

    private fun updateChart(odom: OdomData) {
        val data = binding.chartVelocity.data ?: return
        val x = chartEntryIndex++

        data.getDataSetByIndex(0).addEntry(Entry(x, odom.vx))
        data.getDataSetByIndex(1).addEntry(Entry(x, odom.vy))
        data.getDataSetByIndex(2).addEntry(Entry(x, odom.vz))

        data.notifyDataChanged()
        binding.chartVelocity.notifyDataSetChanged()
        binding.chartVelocity.moveViewToX(x)
    }
    
    private fun updateConnectionStatus(state: ROSBridgeManager.ConnectionState) {
        val (text, colorRes) = when (state) {
            ROSBridgeManager.ConnectionState.CONNECTED -> "Connected" to R.drawable.shape_circle_green
            ROSBridgeManager.ConnectionState.CONNECTING -> "Connecting..." to R.drawable.shape_circle_orange
            ROSBridgeManager.ConnectionState.DISCONNECTED -> "Disconnected" to R.drawable.shape_circle_red
            ROSBridgeManager.ConnectionState.ERROR -> "Error" to R.drawable.shape_circle_red
        }
        
        binding.textConnectionStatus.text = text
        binding.statusIndicator.setBackgroundResource(colorRes)
    }
}
