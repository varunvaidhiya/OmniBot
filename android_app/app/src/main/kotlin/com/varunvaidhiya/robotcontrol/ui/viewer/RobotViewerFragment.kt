package com.varunvaidhiya.robotcontrol.ui.viewer

import android.os.Bundle
import android.view.LayoutInflater
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.varunvaidhiya.robotcontrol.R
import com.varunvaidhiya.robotcontrol.data.models.OdomData
import com.varunvaidhiya.robotcontrol.databinding.FragmentRobotViewerBinding
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager
import com.varunvaidhiya.robotcontrol.ui.common.BaseFragment
import dagger.hilt.android.AndroidEntryPoint
import io.github.sceneview.SceneView
import io.github.sceneview.math.Position
import io.github.sceneview.math.Rotation
import io.github.sceneview.node.ModelNode
import io.github.sceneview.node.Node
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import timber.log.Timber
import kotlin.math.roundToInt

/**
 * 3D robot viewer powered by SceneView (Google Filament).
 */
@AndroidEntryPoint
class RobotViewerFragment : BaseFragment<FragmentRobotViewerBinding>() {

    private val viewModel: RobotViewerViewModel by viewModels()

    private var robotModelNode: ModelNode? = null
    private var goalMarkerNode: Node? = null
    private var modelLoaded = false

    // URDF link names in joint order for the SO-101 arm
    private val armLinkNames = listOf(
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper"
    )

    override fun inflateBinding(
        inflater: LayoutInflater,
        container: ViewGroup?
    ): FragmentRobotViewerBinding =
        FragmentRobotViewerBinding.inflate(inflater, container, false)

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        setupScene()
        setupTapGesture()
        setupObservers()
    }

    private fun setupScene() {
        // Safe check for binding before accessing sceneView
        val sceneView = try { binding.sceneView } catch (e: Exception) { return }

        // Position the default camera at an isometric-ish angle
        sceneView.cameraNode.position = Position(x = 0f, y = 1.5f, z = 2.0f)
        sceneView.cameraNode.lookAt(Position(0f, 0.3f, 0f))

        loadRobotModel(sceneView)
    }

    private fun loadRobotModel(sceneView: SceneView) {
        val assetFiles = try { requireContext().assets.list("") ?: emptyArray() } catch (e: Exception) { emptyArray() }
        if ("robot.glb" !in assetFiles) {
            Timber.i("robot.glb not found in assets")
            // Safe access check
            try {
                binding.layoutNoModel.visibility = View.VISIBLE
                binding.textModelStatus.text = "No model · See banner"
            } catch (e: Exception) {}
            return
        }

        try {
            binding.layoutNoModel.visibility = View.GONE
            binding.textModelStatus.text = "Loading model…"
        } catch (e: Exception) {}

        viewLifecycleOwner.lifecycleScope.launch {
            try {
                // In SceneView 2.2.1, we load the model and then create the node.
                val model = sceneView.modelLoader.loadModel("robot.glb")
                val modelInstance = model?.let { sceneView.modelLoader.createInstance(it) }
                
                if (modelInstance != null) {
                    val modelNode = ModelNode(modelInstance)
                    modelLoaded = true
                    
                    // Check if we still have a view before updating UI or adding child
                    if (_binding != null) {
                        binding.textModelStatus.text = "Model loaded"
                        sceneView.addChildNode(modelNode)
                        robotModelNode = modelNode
                        Timber.i("robot.glb loaded successfully")
                    }
                } else {
                    throw Exception("Failed to create model instance")
                }
            } catch (e: Exception) {
                Timber.e(e, "Failed to load robot.glb")
                if (_binding != null) {
                    binding.textModelStatus.text = "Load error"
                    binding.layoutNoModel.visibility = View.VISIBLE
                }
            }
        }
    }

    private fun setupTapGesture() {
        binding.sceneView.setOnTouchListener { _, event ->
            if (event.action == MotionEvent.ACTION_UP) {
                handleTap(event.x, event.y)
            }
            false
        }
    }

    private fun handleTap(screenX: Float, screenY: Float) {
        try {
            val ray = binding.sceneView.cameraNode.screenPointToRay(screenX, screenY)
            val dir = ray.direction
            val orig = ray.origin
            if (kotlin.math.abs(dir.y) < 1e-6f) return

            val t = -orig.y / dir.y
            if (t < 0) return

            val worldX = orig.x + t * dir.x
            val worldZ = orig.z + t * dir.z

            viewModel.sendNavigationGoal(worldX, -worldZ)
            placeGoalMarker(worldX, 0f, worldZ)
        } catch (e: Exception) {
            Timber.w(e, "Tap hit-test failed")
        }
    }

    private fun placeGoalMarker(x: Float, y: Float, z: Float) {
        // Safe check for binding/sceneView
        val sceneView = try { binding.sceneView } catch (e: Exception) { return }
        
        goalMarkerNode?.let { sceneView.removeChildNode(it) }

        val marker = Node(sceneView.engine).apply {
            position = Position(x, y + 0.05f, z)
        }
        sceneView.addChildNode(marker)
        goalMarkerNode = marker
    }

    private fun setupObservers() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {
                launch {
                    viewModel.connectionState.collectLatest { state ->
                        if (_binding == null) return@collectLatest
                        val dotRes = when (state) {
                            ROSBridgeManager.ConnectionState.CONNECTED   -> R.drawable.shape_circle_green
                            ROSBridgeManager.ConnectionState.CONNECTING  -> R.drawable.shape_circle_orange
                            else                                          -> R.drawable.shape_circle_red
                        }
                        binding.viewerStatusDot.setBackgroundResource(dotRes)
                    }
                }
                launch {
                    viewModel.odomData.collectLatest { odom ->
                        if (_binding == null) return@collectLatest
                        updateRobotPose(odom)
                        updatePoseOverlay(odom)
                    }
                }
                launch {
                    viewModel.armJointPositions.collectLatest { joints ->
                        if (_binding == null) return@collectLatest
                        updateJointOverlay(joints)
                        if (modelLoaded) applyJointTransforms(joints)
                    }
                }
            }
        }
    }

    private fun updateRobotPose(odom: OdomData) {
        val model = robotModelNode ?: return
        model.position = Position(x = odom.x, y = 0f, z = -odom.y)
        model.rotation = Rotation(y = Math.toDegrees(odom.yaw.toDouble()).toFloat())
    }

    private fun applyJointTransforms(joints: DoubleArray) {
        val model = robotModelNode ?: return
        armLinkNames.forEachIndexed { i, name ->
            if (i >= joints.size) return@forEachIndexed
            val angle = joints[i].toFloat()
            val node = findNodeByName(model, name) ?: return@forEachIndexed
            val rotationDeg = Math.toDegrees(angle.toDouble()).toFloat()
            node.rotation = when (i) {
                0 -> Rotation(y = rotationDeg)
                else -> Rotation(x = rotationDeg)
            }
        }
    }

    private fun findNodeByName(root: Node, name: String): Node? {
        if (root.name == name) return root
        for (child in root.childNodes) {
            val found = findNodeByName(child, name)
            if (found != null) return found
        }
        return null
    }

    private fun updatePoseOverlay(odom: OdomData) {
        try {
            binding.textViewerX.text   = "X: %.2f".format(odom.x)
            binding.textViewerY.text   = "Y: %.2f".format(odom.y)
            binding.textViewerYaw.text = "θ: ${Math.toDegrees(odom.yaw.toDouble()).roundToInt()}°"
        } catch (e: Exception) {}
    }

    private fun updateJointOverlay(joints: DoubleArray) {
        try {
            val sb = StringBuilder()
            joints.forEachIndexed { i, rad ->
                if (i > 0) sb.append("  ")
                sb.append("${Math.toDegrees(rad).roundToInt()}°")
            }
            binding.textJointValues.text = sb.toString()
        } catch (e: Exception) {}
    }

    override fun onDestroyView() {
        // Clean up nodes before the View is destroyed to avoid Filament engine leaks
        // during fast tab switching.
        try {
            val sceneView = binding.sceneView
            robotModelNode?.let { sceneView.removeChildNode(it) }
            goalMarkerNode?.let { sceneView.removeChildNode(it) }
        } catch (e: Exception) {
            // Binding might already be null or View detached
        }
        
        robotModelNode = null
        goalMarkerNode = null
        modelLoaded = false

        super.onDestroyView()
    }
}
