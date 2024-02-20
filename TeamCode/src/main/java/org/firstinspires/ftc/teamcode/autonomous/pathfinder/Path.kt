package org.firstinspires.ftc.teamcode.autonomous.pathfinder

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase.*
import kotlin.math.hypot

internal const val squareSize = 584.2 // mm

internal const val motorEncoderEventsPerRevolution = 537.7
internal const val wheelCircumference = 310 // mm
const val motorEncoderEventsPerMM = motorEncoderEventsPerRevolution / wheelCircumference

internal const val strafingCoefficient = 0.7071 // speed difference between normal running and strafing - 1/sqrt(2), approximately


interface Segment<I, R> {
    val name: String
    fun run(driveBase: DriveBase, input: I): R

    /** Input: (x to y), where x is horizontal (strafing) and y is forward/backward */
    object Grid : Segment<Pair<Float, Float>, Unit> {
        override val name = "Grid"

        override fun run(driveBase: DriveBase, input: Pair<Float, Float>) {
            val x = input.first
            val y = input.second
            val xTicks = x * squareSize * motorEncoderEventsPerMM
            val yTicks = y * squareSize * motorEncoderEventsPerMM
            val segment = this
            RobotLog.i("Running a Segment")
            driveBase.run {
                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
                val coefficients = mecanumCoefficientsForDirection(with(segment) {
                    if(x < 0 && y < 0) {
                        TravelDirection.STRAFE_LEFT_BACKWARD
                    } else if(x < 0 && y > 0) {
                        TravelDirection.STRAFE_LEFT_FORWARD
                    } else if(x > 0 && y < 0) {
                        TravelDirection.STRAFE_RIGHT_BACKWARD
                    } else if(x > 0 && y > 0) {
                        TravelDirection.STRAFE_RIGHT_FORWARD
                    } else if(x > 0) {
                        TravelDirection.STRAFE_RIGHT
                    } else if(x < 0) {
                        TravelDirection.STRAFE_LEFT
                    } else if(y > 0) {
                        TravelDirection.FORWARD
                    } else if(y < 0) {
                        TravelDirection.REVERSE
                    } else {
                        TravelDirection.BASE
                    }
                })
                RobotLog.i("Pathfinder/Segment:Grid", "X: $xTicks, Y: $yTicks - total ticks: ${hypot(xTicks, yTicks)}")
                go(0.5, coefficients.map { coefficient -> (coefficient * hypot(xTicks, yTicks)).toInt() }.toTypedArray(), 1)
                while(motors.any { it.isBusy }) {}
            }
        }
    }

    object Yaw : Segment<Double, Unit> {
        override val name = "Yaw"
        override fun run(driveBase: DriveBase, input: Double) {
            driveBase.yaw(input, 0.2)
        }
    }

    class Run<I, R>(val closure: (DriveBase, I) -> R) : Segment<I, R> {
        override val name = "Run"
        override fun run(driveBase: DriveBase, input: I) = closure(driveBase, input)
    }

    object Noop : Segment<Unit, Unit> {
        override val name = "No-op"
        override fun run(driveBase: DriveBase, input: Unit) {

        }
    }
}

operator fun <I, R, N> Segment<I, R>.div(other: Segment<R, N>): Segment<I, N> {
    val self = this
    return object : Segment<I, N> {
        override val name: String
            get() = "${self.name} -> ${other.name}"

        override fun run(driveBase: DriveBase, input: I): N
            = other.run(driveBase, self.run(driveBase, input))
    }
}

operator fun <I, R> I.div(segment: Segment<I, R>): Segment<Unit, R> {
    val self = this
    return object : Segment<Unit, R> {
        override val name: String
            get() = "=> ${segment.name}"

        override fun run(driveBase: DriveBase, input: Unit) =
            segment.run(driveBase, self)
    }
}

operator fun <I, N> Segment<I, Unit>.rangeTo(other: Segment<Unit, N>) : Segment<I, N> {
    val self = this
    return object : Segment<I, N> {
        override val name: String
            get() = "${self.name}; ${other.name}"

        override fun run(driveBase: DriveBase, input: I): N {
            self.run(driveBase, input)
            return other.run(driveBase, Unit)
        }
    }
}