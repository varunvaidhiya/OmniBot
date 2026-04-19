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
import com.google.ar.sceneform.math.Quaternion
import com.google.ar.sceneform.math.Vector3
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
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * 3D robot viewer powered by SceneView (Google Filament).
 *
 * Loads robot.glb from assets/ if available.
 * When the GLB is present, node names must match the URDF link names so joint
 * transforms can be applied at runtime.  Run tools/urdf_to_glb.py to generate
 * the GLB from the repository URDF + STL meshes.
 *
 * When no GLB is found the scene still shows:
 *   - A dark grid floor plane
 *   - The robot's world position from odometry (marker on floor)
 *   - Live arm joint angles in the bottom overlay
 *   - Tap-to-navigate gesture
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

    // Approximate DH translation offsets (metres) for each joint from URDF
    // [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
    private val linkOffsets = listOf(
        Position(0f, 0.065f, 0f),
        Position(0f, 0.060f, 0f),
        Position(0f, 0.100f, 0f),
        Position(0f, 0.095f, 0f),
        Position(0f, 0.055f, 0f),
        Position(0f, 0.040f, 0f)
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

    // ── Scene setup ─────────────────────────────────────────────────────────

    private fun setupScene() {
        val sceneView = binding.sceneView

        // Default lighting — indirect via IBL + sun directional light
        sceneView.apply {
            // Position the default camera at an isometric-ish angle
            try {
                camera.apply {
                    position = Position(x = 0f, y = 1.5f, z = 2.0f)
                    lookAt(Position(0f, 0.3f, 0f))
                }
            } catch (e: Exception) {
                Timber.w(e, "Camera config not available on this SceneView version")
            }
        }

        // Try to load the GLB model
        loadRobotModel(sceneView)
    }

    private fun loadRobotModel(sceneView: SceneView) {
        // Check if robot.glb exists in assets
        val assetFiles = try { requireContext().assets.list("") ?: emptyArray() } catch (e: Exception) { emptyArray() }
        if ("robot.glb" !in assetFiles) {
            Timber.i("robot.glb not found in assets — showing no-model banner")
            binding.layoutNoModel.visibility = View.VISIBLE
            binding.textModelStatus.text = "No model · See banner"
            return
        }

        binding.layoutNoModel.visibility = View.GONE
        binding.textModelStatus.text = "Loading model…"

        val modelNode = ModelNode(
            engine = sceneView.engine,
            autoAnimate = false
        )

        viewLifecycleOwner.lifecycleScope.launch {
            try {
                modelNode.loadModelGlbAsync(
                    context = requireContext(),
                    glbFileLocation = "robot.glb",
                    centerOrigin = null
                ) {
                    modelLoaded = true
                    binding.textModelStatus.text = "Model loaded"
                    Timber.i("robot.glb loaded successfully")
                }
                sceneView.addChildNode(modelNode)
                robotModelNode = modelNode
            } catch (e: Exception) {
                Timber.e(e, "Failed to load robot.glb")
                binding.textModelStatus.text = "Load error"
                binding.layoutNoModel.visibility = View.VISIBLE
            }
        }
    }

    // ── Tap-to-navigate ────────────────────────────────────────────────────

    private fun setupTapGesture() {
        binding.sceneView.setOnTouchListener { _, event ->
            if (event.action == MotionEvent.ACTION_UP) {
                handleTap(event.x, event.y)
            }
            // Pass through so SceneView orbit gesture still works
            false
        }
    }

    private fun handleTap(screenX: Float, screenY: Float) {
        // Perform a hit-test against the floor plane (y ≈ 0)
        // SceneView's camera ray-cast against y=0 plane
        try {
            val ray = binding.sceneView.camera.screenPointToRay(screenX, screenY)

            // Intersect with y=0 plane: t = -ray.origin.y / ray.direction.y
            val dir = ray.direction
            val orig = ray.origin
            if (kotlin.math.abs(dir.y) < 1e-6f) return  // parallel to floor

            val t = -orig.y / dir.y
            if (t < 0) return  // behind camera

            val worldX = orig.x + t * dir.x
            val worldZ = orig.z + t * dir.z   // ROS Y = screen -Z

            Timber.i("Nav goal: world (%.2f, %.2f)".format(worldX, -worldZ))
            viewModel.sendNavigationGoal(worldX, -worldZ)

            // Show goal marker
            placeGoalMarker(worldX, 0f, worldZ)
        } catch (e: Exception) {
            Timber.w(e, "Tap hit-test failed")
        }
    }

    private fun placeGoalMarker(x: Float, y: Float, z: Float) {
        goalMarkerNode?.detachFromParent()

        val marker = Node(engine = binding.sceneView.engine).apply {
            position = Position(x, y + 0.05f, z)
        }
        binding.sceneView.addChildNode(marker)
        goalMarkerNode = marker
    }

    // ── Live data observers ──────────────────────────────────────────────

    private fun setupObservers() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewLifecycleOwner.repeatOnLifecycle(Lifecycle.State.STARTED) {

                // Connection dot
                launch {
                    viewModel.connectionState.collectLatest { state ->
                        val dotRes = when (state) {
                            ROSBridgeManager.ConnectionState.CONNECTED   -> R.drawable.shape_circle_green
                            ROSBridgeManager.ConnectionState.CONNECTING  -> R.drawable.shape_circle_orange
                            else                                          -> R.drawable.shape_circle_red
                        }
                        binding.viewerStatusDot.setBackgroundResource(dotRes)
                    }
                }

                // Robot world pose — move model node on the floor plane
                launch {
                    viewModel.odomData.collectLatest { odom ->
                        updateRobotPose(odom)
                        updatePoseOverlay(odom)
                    }
                }

                // Arm joints — animate bones when model is loaded
                launch {
                    viewModel.armJointPositions.collectLatest { joints ->
                        updateJointOverlay(joints)
                        if (modelLoaded) applyJointTransforms(joints)
                    }
                }
            }
        }
    }

    private fun updateRobotPose(odom: OdomData) {
        val model = robotModelNode ?: return
        // ROS: X=forward, Y=left, Z=up → SceneView: X=right, Y=up, Z=backward
        // Mapping: ros.x → sv.x, ros.y → sv.-z, ros.yaw → rotation about Y
        model.position = Position(x = odom.x, y = 0f, z = -odom.y)
        model.rotation = Rotation(y = Math.toDegrees(odom.yaw.toDouble()).toFloat())
    }

    /**
     * Apply forward kinematics to each named arm link node in the loaded GLB.
     * The GLB must have nodes named exactly after the URDF link names
     * (generated by tools/urdf_to_glb.py).
     */
    private fun applyJointTransforms(joints: DoubleArray) {
        val model = robotModelNode ?: return

        armLinkNames.forEachIndexed { i, name ->
            if (i >= joints.size) return@forEachIndexed
            val angle = joints[i].toFloat()   // radians

            // Find the node in the loaded GLB hierarchy by name
            val node = findNodeByName(model, name) ?: return@forEachIndexed

            // Each SO-101 joint rotates about Z (yaw for pan, pitch for lift etc.)
            // Adjust axis per joint type if needed once GLB export is validated
            val rotationDeg = Math.toDegrees(angle.toDouble()).toFloat()
            node.rotation = when (i) {
                0 -> Rotation(y = rotationDeg)    // shoulder_pan  — yaw
                else -> Rotation(x = rotationDeg) // all others    — pitch
            }
        }
    }

    /** Recursive search for a node by name inside a ModelNode hierarchy. */
    private fun findNodeByName(root: Node, name: String): Node? {
        if (root.name == name) return root
        for (child in root.childNodes) {
            val found = findNodeByName(child, name)
            if (found != null) return found
        }
        return null
    }

    // ── Overlay text updates ─────────────────────────────────────────────

    private fun updatePoseOverlay(odom: OdomData) {
        binding.textViewerX.text   = "X: %.2f".format(odom.x)
        binding.textViewerY.text   = "Y: %.2f".format(odom.y)
        binding.textViewerYaw.text = "θ: ${Math.toDegrees(odom.yaw.toDouble()).roundToInt()}°"
    }

    private fun updateJointOverlay(joints: DoubleArray) {
        val sb = StringBuilder()
        joints.forEachIndexed { i, rad ->
            if (i > 0) sb.append("  ")
            sb.append("${Math.toDegrees(rad).roundToInt()}°")
        }
        binding.textJointValues.text = sb.toString()
    }

    override fun onDestroyView() {
        // SceneView manages its own Filament lifecycle via the fragment lifecycle observer
        // but we clean up our nodes explicitly
        robotModelNode?.detachFromParent()
        goalMarkerNode?.detachFromParent()
        super.onDestroyView()
    }
}
