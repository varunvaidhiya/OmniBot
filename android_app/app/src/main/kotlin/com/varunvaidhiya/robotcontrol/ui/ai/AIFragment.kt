package com.varunvaidhiya.robotcontrol.ui.ai

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.EditorInfo
import androidx.fragment.app.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.recyclerview.widget.LinearLayoutManager
import com.varunvaidhiya.robotcontrol.R
import com.varunvaidhiya.robotcontrol.databinding.FragmentAiBinding
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

@AndroidEntryPoint
class AIFragment : BaseFragment<FragmentAiBinding>() {

    private val aiViewModel: AIViewModel by viewModels()
    private val chatAdapter = ChatAdapter()

    override fun inflateBinding(inflater: LayoutInflater, container: ViewGroup?): FragmentAiBinding =
        FragmentAiBinding.inflate(inflater, container, false)

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        setupRecycler()
        setupInput()
        setupQuickChips()
        setupRecordingSwitch()
        setupObservers()
    }

    private fun setupRecycler() {
        val lm = LinearLayoutManager(requireContext()).apply { stackFromEnd = true }
        binding.recyclerMessages.layoutManager = lm
        binding.recyclerMessages.adapter = chatAdapter
    }

    private fun setupInput() {
        binding.editCommand.setOnEditorActionListener { _, actionId, _ ->
            if (actionId == EditorInfo.IME_ACTION_SEND) {
                sendCommand(); true
            } else false
        }
        binding.btnSend.setOnClickListener { sendCommand() }
    }

    private fun setupQuickChips() {
        binding.chipNavigate.setOnClickListener { prefill("Navigate home") }
        binding.chipExplore.setOnClickListener  { prefill("Explore the area") }
        binding.chipPick.setOnClickListener     { prefill("Pick up the object") }
        binding.chipStop.setOnClickListener     { aiViewModel.sendCommand("stop") }
    }

    private fun setupRecordingSwitch() {
        binding.switchRecording.setOnCheckedChangeListener { _, checked ->
            aiViewModel.toggleRecording(checked)
            binding.textRecordingStatus.text =
                if (checked) "Dataset recording: ON  ⏺" else "Dataset recording: OFF"
        }
    }

    private fun setupObservers() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {

                launch {
                    aiViewModel.messages.collectLatest { msgs ->
                        chatAdapter.submitList(msgs)
                        binding.recyclerMessages.scrollToPosition(msgs.size - 1)
                    }
                }

                launch {
                    aiViewModel.connectionState.collectLatest { state ->
                        val dotRes = when (state) {
                            ROSBridgeManager.ConnectionState.CONNECTED   -> R.drawable.shape_circle_green
                            ROSBridgeManager.ConnectionState.CONNECTING  -> R.drawable.shape_circle_orange
                            else                                          -> R.drawable.shape_circle_red
                        }
                        binding.aiStatusDot.setBackgroundResource(dotRes)
                    }
                }

                launch {
                    aiViewModel.isRecording.collectLatest { rec ->
                        binding.switchRecording.isChecked = rec
                        binding.textRecordingStatus.text =
                            if (rec) "Dataset recording: ON  ⏺" else "Dataset recording: OFF"
                    }
                }
            }
        }
    }

    private fun sendCommand() {
        val text = binding.editCommand.text?.toString()?.trim() ?: return
        if (text.isEmpty()) return
        aiViewModel.sendCommand(text)
        binding.editCommand.setText("")
    }

    private fun prefill(text: String) {
        binding.editCommand.setText(text)
        binding.editCommand.setSelection(text.length)
    }
}
