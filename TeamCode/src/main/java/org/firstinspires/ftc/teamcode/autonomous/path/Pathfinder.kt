package org.firstinspires.ftc.teamcode.autonomous.path

import org.firstinspires.ftc.teamcode.subassemblies.DriveBase

internal const val squareSize = 584.2 // mm

internal const val motorEncoderEventsPerRevolution = 537.7
internal const val wheelCircumference = 310 // mm
internal const val motorEncoderEventsPerMM = motorEncoderEventsPerRevolution / wheelCircumference

fun DriveBase.runGrid(x: Double, y: Double, yaw: Double, curve: Boolean = false) {
    val xTicks = (x * squareSize * motorEncoderEventsPerMM)
    val yTicks = y * squareSize * motorEncoderEventsPerMM


}