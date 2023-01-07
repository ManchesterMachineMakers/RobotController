package org.firstinspires.ftc.teamcode.util.pathfinder

import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.DriveBase

/**
 * @param x Horizontal (strafing) distance in MM
 * @param y Vertical (forward/back) distance in MM
 */
data class Segment(val x: Float, val y: Float) {
    val xTicks = x * DriveBase.motorEncoderEventsPerMM
    val yTicks = y * DriveBase.motorEncoderEventsPerMM
}

class Path(private vararg val segments: Segment) : Iterable<Segment> {
    override fun iterator() = segments.iterator()
}