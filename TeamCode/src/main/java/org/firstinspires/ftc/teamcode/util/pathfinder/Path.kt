package org.firstinspires.ftc.teamcode.util.pathfinder

import java.lang.Math.toDegrees
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

class Path(originX: Float, originY: Float, targetX: Float, targetY: Float, initHeading: Double, clearanceRadius: Float) {
    val heading: Double
    val reverseHeading: Double
    val distance: Double
    val rotation: Double
    val direction: Double

    init {
        val deltaX = targetX - originX
        val deltaY = targetY - originY
        val newHeading = toDegrees(atan2(deltaY, deltaX).toDouble()) + 90
        val oppHeading = if (newHeading < 180) newHeading + 180 else newHeading - 180

        // the distance from origin to target, minus the clearance radius.
        distance = sqrt(deltaX * deltaX + deltaY * deltaY).toDouble() - clearanceRadius

        val rot = arrayOf(
            (360 - initHeading + newHeading) % 360,
            -(360 - newHeading + initHeading) % 360,
        )
        // This is the least rotation (leastRot in Isaac's code)
        //TODO: where is the absolute value comparison?  You need to know which way to turn,
        // so which direction is the most efficient, and then preserve the actual
        // rotational value so that you go the right way.
        // see code in Pathfinder.kt
        rotation = rot.minOf { it }
        direction = 1.0
        // for going forwards
        heading = initHeading + rotation
        // for going backwards
        reverseHeading = if (heading < 180) heading + 180 else heading - 180
    }
}