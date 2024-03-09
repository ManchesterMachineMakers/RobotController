package org.firstinspires.ftc.teamcode.autonomous.pathfinder

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * @param l radius in mm
 * @param theta angle in radians
 */
fun encodersFromPolar(l: Double, theta: Double) =
        arrayOf(
                cos(theta) - sin(theta),
                cos(theta) + sin(theta),
                cos(theta) + sin(theta),
                cos(theta) - sin(theta)
        )
                .map { it * (motorEncoderEventsPerRevolution*l)/(wheelCircumference/(2*PI)) }
                .toTypedArray()

fun DriveBase.runPolar(telemetry: Telemetry, power: Double, l: Double, theta: Double) =
        encodersFromPolar(l, theta).let {
            it.forEachIndexed { index, ticks -> telemetry.addData("Ticks $index", ticks) }
            // Standardize power levels based on left front
            val powerLevels = arrayOf(
                    1.0,
                    it[1] / it[0],
                    it[2] / it[0],
                    it[3] / it[0]
            )
                    .map { it * 0.6 }
                    .toTypedArray()

            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            val ticks = it.map { it.roundToInt() }
            setTargetPositions(ticks[0], ticks[1], ticks[2], ticks[3])
            setMode(DcMotor.RunMode.RUN_TO_POSITION)
            setPower(powerLevels)
        }