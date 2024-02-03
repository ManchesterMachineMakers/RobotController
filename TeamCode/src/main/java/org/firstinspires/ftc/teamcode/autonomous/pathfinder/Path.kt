package org.firstinspires.ftc.teamcode.autonomous.pathfinder

import android.util.Log
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


interface Segment {
    val name: String
    fun run(driveBase: DriveBase)

    /**
     * @param x Horizontal (strafing) distance in MM
     * @param y Vertical (forward/back) distance in MM
     */
    data class Grid(val x: Float, val y: Float) : Segment {
        val xTicks get() = x * squareSize * motorEncoderEventsPerMM
        val yTicks get() = y * squareSize * motorEncoderEventsPerMM
        override val name = "Grid"

        override fun run(driveBase: DriveBase) {
            val segment = this
            driveBase.run {
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
                Log.i("Pathfinder/Segment:Grid", "X: ${segment.xTicks}, Y: ${segment.yTicks} - total ticks: ${hypot(segment.xTicks, segment.yTicks)}")
                go(0.5, coefficients.map { coefficient -> (coefficient * hypot(segment.xTicks, segment.yTicks)).toInt() }.toTypedArray(), 1)
                while(motors.any { it.isBusy }) {}
            }
        }
    }

    data class Yaw(val degrees: Double) : Segment {
        override val name = "Yaw"
        override fun run(driveBase: DriveBase) {
            driveBase.yaw(degrees, 0.2)
        }
    }

    class Noop : Segment {
        override val name = "No-op"
        override fun run(driveBase: DriveBase) {

        }
    }
}

class Path(private vararg val segments: Segment) : Iterable<Segment> {
    override fun iterator() = segments.iterator()
}

fun DriveBase.runPath(path: Path) {
    for (segment in path) {
        Log.i("Pathfinder", "Running step: ${segment.name}")
        segment.run(this)
    }
}