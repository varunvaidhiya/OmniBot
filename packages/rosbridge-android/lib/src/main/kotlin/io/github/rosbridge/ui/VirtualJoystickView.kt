package io.github.rosbridge.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.*

/**
 * A circular virtual joystick with configurable deadzone.
 *
 * Outputs normalized (x, y) values in [-1, 1] × [-1, 1].
 * Y is inverted so that up = positive (matching ROS REP 103 +x forward convention).
 *
 * Usage (XML):
 * ```xml
 * <io.github.rosbridge.ui.VirtualJoystickView
 *     android:layout_width="200dp"
 *     android:layout_height="200dp" />
 * ```
 *
 * Usage (code):
 * ```kotlin
 * joystick.onJoystickMoved = { x, y ->
 *     // x: strafe, y: forward
 *     publishCmdVel(linear_x = y, linear_y = -x)
 * }
 * ```
 *
 * @param deadzone  Fraction of base radius treated as zero (default 0.10 = 10%).
 * @param baseColor Color of the outer circle (default light gray).
 * @param hatColor  Color of the inner thumb (default Material Blue 500).
 */
class VirtualJoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : View(context, attrs, defStyleAttr) {

    var deadzone: Float = 0.10f
    var onJoystickMoved: ((x: Float, y: Float) -> Unit)? = null

    private var cx = 0f
    private var cy = 0f
    private var baseRadius = 0f
    private var hatRadius = 0f
    private var hatX = 0f
    private var hatY = 0f

    private val basePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.LTGRAY
        style = Paint.Style.STROKE
        strokeWidth = 5f
        alpha = 140
    }
    private val hatPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#2196F3")
        style = Paint.Style.FILL
        alpha = 200
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        cx = w / 2f
        cy = h / 2f
        baseRadius = minOf(w, h) / 2.5f
        hatRadius  = baseRadius / 3f
        resetHat()
    }

    override fun onDraw(canvas: Canvas) {
        canvas.drawCircle(cx, cy, baseRadius, basePaint)
        canvas.drawCircle(hatX, hatY, hatRadius, hatPaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN,
            MotionEvent.ACTION_MOVE -> {
                val dx = event.x - cx
                val dy = event.y - cy
                val dist = sqrt(dx * dx + dy * dy)
                if (dist <= baseRadius) {
                    hatX = event.x
                    hatY = event.y
                } else {
                    val ratio = baseRadius / dist
                    hatX = cx + dx * ratio
                    hatY = cy + dy * ratio
                }
                notify()
                invalidate()
            }
            MotionEvent.ACTION_UP,
            MotionEvent.ACTION_CANCEL -> {
                resetHat()
                notify()
                invalidate()
            }
        }
        return true
    }

    private fun resetHat() {
        hatX = cx
        hatY = cy
    }

    private fun notify() {
        var normX =  (hatX - cx) / baseRadius
        var normY = -(hatY - cy) / baseRadius   // invert Y for ROS convention
        val mag = sqrt(normX * normX + normY * normY)
        if (mag < deadzone) { normX = 0f; normY = 0f }
        onJoystickMoved?.invoke(normX, normY)
    }
}
