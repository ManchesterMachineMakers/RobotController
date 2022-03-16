package org.firstinspires.ftc.teamcode.util.pathfinder

import java.lang.Math.toDegrees
import kotlin.math.atan2
import kotlin.math.sqrt

class Path(originX: Float, originY: Float, targetX: Float, targetY: Float, initHeading: Double, clearanceRadius: Float) {
    val heading: Double
    val distance: Double
    val rotation: Double
    val direction: Double

    init {
        val deltaX = targetX - originX
        val deltaY = targetY - originY
        val newHeading = toDegrees(atan2(deltaY, deltaX).toDouble()) + 90
        val oppHeading = if (newHeading < 180) newHeading + 180 else newHeading - 180
        distance = sqrt(deltaX * deltaX + deltaY * deltaY).toDouble() - clearanceRadius
        val rot = arrayOf(
            (360 - initHeading + newHeading) % 360,
            -(360 - newHeading + initHeading) % 360,
        )
        // This is the least rotation (leastRot in Isaac's code)
        rotation = rot.minOf { it }
        direction = 1.0
        heading = initHeading + rotation
    }
}