package org.firstinspires.ftc.teamcode.autonomous.path

import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.roundToInt

internal const val squareSize = 584.2 // mm

internal const val motorEncoderEventsPerRevolution = 537.7
internal const val wheelCircumference = 310 // mm
const val motorEncoderEventsPerMM = motorEncoderEventsPerRevolution / wheelCircumference

internal const val strafingCoefficient = 0.7071 // speed difference between normal running and strafing - 1/sqrt(2), approximately

data class GridPath(val x: Double, val y: Double, val yaw: Double)

fun DriveBase.runGrid(strafe: Double, fwb: Double, yaw: Double) {
    val y = strafe
    val x = fwb
    val xTicks = x * squareSize * motorEncoderEventsPerMM
    val yTicks = y * squareSize * motorEncoderEventsPerMM * (1/strafingCoefficient)
    val yawTicks = yaw * squareSize * motorEncoderEventsPerMM

    val (_, _, _, leftFront, rightFront, leftRear, rightRear) = DriveBase.MoveRobotCalculations(xTicks, yTicks, yawTicks)

    runToPosition(leftFront.roundToInt(), rightFront.roundToInt(), leftRear.roundToInt(), rightRear.roundToInt())
    while(motors.any { it.isBusy }) {}
}

fun DriveBase.runGrid(path: GridPath) {
    runGrid(path.x, path.y, path.yaw)
}