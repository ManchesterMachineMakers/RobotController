package org.firstinspires.ftc.teamcode.autonomous.pathfinder

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * @param l radius in mm
 * @param theta angle in radians
 */
fun encodersFromPolar(l: Double, theta: Double): Array<Double> {
    val r = wheelCircumference/(2*PI)

    return arrayOf(
        cos(theta) - sin(theta),
        cos(theta) + sin(theta),
        cos(theta) + sin(theta),
        cos(theta) - sin(theta)
    )
        .map { (motorEncoderEventsPerRevolution*l / r) * it * -1 }
        .toTypedArray()
}

fun DriveBase.runPolar(telemetry: Telemetry, power: Double, l: Double, theta: Double) =
        encodersFromPolar(l, theta).let {
            it.forEachIndexed { index, ticks -> telemetry.addData("Ticks $index", ticks) }
            val absValues = it.map { it.absoluteValue }
            // Standardize power levels based on left front
            val highestPower = absValues.maxOrNull() ?: 1.0
            val powerLevels = arrayOf(
                    absValues[0] / highestPower,
                    absValues[1] / highestPower,
                    absValues[2] / highestPower,
                    absValues[3] / highestPower
            )
                    .map { value -> value * power * (if(it.average() < 0) -1 else 1) }
                    .toTypedArray()

            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            Thread.yield()
            val ticks = it.map { it.roundToInt() }
            motors.forEach { (it as DcMotorEx).targetPositionTolerance = 6 }
            setTargetPositions(ticks[0], ticks[1], ticks[2], ticks[3])
            setMode(DcMotor.RunMode.RUN_TO_POSITION)
            setPower(powerLevels)
        }

fun DriveBase.runPolarAndWait(opModeIsActive: () -> Boolean, telemetry: Telemetry, power: Double, l: Double, theta: Double) {
    runPolar(telemetry, power, l, theta)
    telemetry.addData("Motors-Target", motors.map { it.targetPosition })
    telemetry.addData("Motors-Actual", motors.map { it.currentPosition })
    telemetry.update()
    Thread.yield()
    while (opModeIsActive() && motors.any {it.isBusy}) {
        telemetry.addData("Motors-Target", motors.map { it.targetPosition })
        telemetry.addData("Motors-Actual", motors.map { it.currentPosition })
        telemetry.update()
        Thread.yield()
    }
}