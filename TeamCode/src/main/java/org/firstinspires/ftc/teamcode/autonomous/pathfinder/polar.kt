package org.firstinspires.ftc.teamcode.autonomous.pathfinder

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
                .map { it * (motorEncoderEventsPerMM*l)/(wheelCircumference/(2*PI)) }
                .map { it.roundToInt() }
                .toTypedArray()

fun DriveBase.runPolar(power: Double, l: Double, theta: Double) =
        this.go(power, encodersFromPolar(l, theta), 6)