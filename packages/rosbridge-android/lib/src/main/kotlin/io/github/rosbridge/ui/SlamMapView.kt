package io.github.rosbridge.ui

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View

/**
 * Renders a ROS OccupancyGrid map with robot pose overlay.
 *
 * Supports pinch-to-zoom and drag-to-pan gestures.
 * Double-tap resets the viewport.
 *
 * Usage:
 * ```kotlin
 * // On /map topic message:
 * slamMapView.updateMap(
 *     width  = msg["info"]["width"] as Int,
 *     height = msg["info"]["height"] as Int,
 *     data   = (msg["data"] as List<Int>).toIntArray(),
 * )
 * // On /amcl_pose topic message:
 * slamMapView.updateRobotPose(poseX, poseY, yaw)
 * ```
 */
class SlamMapView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {

    private var mapBitmap: Bitmap? = null
    private var robotX = 0f
    private var robotY = 0f
    private var robotYaw = 0f

    private var scaleFactor = 1f
    private var transX = 0f
    private var transY = 0f

    private val robotPaint = Paint().apply { color = Color.RED;   style = Paint.Style.FILL }
    private val mapPaint   = Paint(Paint.ANTI_ALIAS_FLAG)

    private val scaleDetector   = ScaleGestureDetector(context, ScaleListener())
    private val gestureDetector = GestureDetector(context, GestureListener())

    /**
     * Update the occupancy grid.
     *
     * @param width  Map width in cells.
     * @param height Map height in cells.
     * @param data   Flat array: -1 = unknown, 0 = free, 100 = occupied.
     */
    fun updateMap(width: Int, height: Int, data: IntArray) {
        if (width <= 0 || height <= 0) return
        if (mapBitmap == null || mapBitmap!!.width != width || mapBitmap!!.height != height) {
            mapBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
        }
        val pixels = IntArray(data.size) { i ->
            when (data[i]) {
                -1   -> Color.GRAY
                100  -> Color.BLACK
                else -> Color.WHITE
            }
        }
        mapBitmap!!.setPixels(pixels, 0, width, 0, 0, width, height)
        postInvalidate()
    }

    /**
     * Update the robot pose overlay (coordinates in map pixels).
     *
     * @param x   X position in map pixel space.
     * @param y   Y position in map pixel space.
     * @param yaw Heading in radians (0 = +x, positive = CCW).
     */
    fun updateRobotPose(x: Float, y: Float, yaw: Float) {
        robotX = x; robotY = y; robotYaw = yaw
        postInvalidate()
    }

    override fun onDraw(canvas: Canvas) {
        canvas.save()
        canvas.translate(transX, transY)
        canvas.scale(scaleFactor, scaleFactor)

        mapBitmap?.let { canvas.drawBitmap(it, 0f, 0f, mapPaint) }

        // Draw robot as a forward-pointing triangle
        canvas.save()
        canvas.translate(robotX, robotY)
        canvas.rotate(Math.toDegrees(robotYaw.toDouble()).toFloat())
        val path = Path().apply {
            moveTo(10f, 0f); lineTo(-5f, 5f); lineTo(-5f, -5f); close()
        }
        canvas.drawPath(path, robotPaint)
        canvas.restore()

        canvas.restore()
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleDetector.onTouchEvent(event)
        gestureDetector.onTouchEvent(event)
        return true
    }

    private inner class ScaleListener : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(d: ScaleGestureDetector): Boolean {
            scaleFactor = (scaleFactor * d.scaleFactor).coerceIn(0.1f, 10f)
            invalidate(); return true
        }
    }

    private inner class GestureListener : GestureDetector.SimpleOnGestureListener() {
        override fun onScroll(e1: MotionEvent?, e2: MotionEvent, dx: Float, dy: Float): Boolean {
            transX -= dx; transY -= dy; invalidate(); return true
        }
        override fun onDoubleTap(e: MotionEvent): Boolean {
            scaleFactor = 1f; transX = 0f; transY = 0f; invalidate(); return true
        }
    }
}
