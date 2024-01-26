package org.firstinspires.ftc.teamcode.autonomous.pathfinder

import com.qualcomm.robotcore.hardware.DcMotor
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase.*
import kotlin.math.hypot

internal const val squareSize = 584.2 // mm

internal const val motorEncoderEventsPerRevolution = 537.7
internal const val wheelCircumference = 310 // mm
const val motorEncoderEventsPerMM = motorEncoderEventsPerRevolution / wheelCircumference

internal const val strafingCoefficient = 0.7071 // speed difference between normal running and strafing - 1/sqrt(2), approximately


/**
 * @param x Horizontal (strafing) distance in MM
 * @param y Vertical (forward/back) distance in MM
 */
data class Segment(val x: Float, val y: Float) {
    val xTicks = x * motorEncoderEventsPerMM
    val yTicks = y * motorEncoderEventsPerMM
}

class Path(private vararg val segments: Segment) : Iterable<Segment> {
    override fun iterator() = segments.iterator()
}

fun DriveBase.runPath(path: Path) {
    for (segment in path) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        val coefficients = mecanumCoefficientsForDirection(with(segment) {
            if(x < 0 && y < 0) {
                TravelDirection.strafeLeftBackward
            } else if(x < 0 && y > 0) {
                TravelDirection.strafeLeftForward
            } else if(x > 0 && y < 0) {
                TravelDirection.strafeRightBackward
            } else if(x > 0 && y > 0) {
                TravelDirection.strafeRightForward
            } else if(x > 0) {
                TravelDirection.strafeRight
            } else if(x < 0) {
                TravelDirection.strafeLeft
            } else if(y > 0) {
                TravelDirection.forward
            } else if(y < 0) {
                TravelDirection.reverse
            } else {
                TravelDirection.base
            }
        })
        go(0.2, coefficients.map { coefficient -> (coefficient * hypot(segment.xTicks, segment.yTicks)).toInt() }.toTypedArray(), 1)
        while(motors.any { it.isBusy }) {}
    }
}