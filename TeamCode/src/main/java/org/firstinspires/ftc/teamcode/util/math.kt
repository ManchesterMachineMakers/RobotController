package org.firstinspires.ftc.teamcode.util

import kotlin.math.*

fun clamp(x: Double, lowerBound: Double, upperBound: Double)
    = max(lowerBound, min(upperBound, x))

fun equalsTolerance(a: Double, b: Double, tolerance: Double)
    = abs(b - a) < tolerance

fun encoderPositionToDegrees(encoderPosition: Int, encoderResolution: Double)
    = 360 * encoderPosition / encoderResolution

/** Assumes use of a 300 degree non-continuous servo */
fun degreesToServoPosition(degrees: Double, scaleRange: Pair<Double, Double>): Double {
    val scale = abs(scaleRange.first - scaleRange.second)
    return degrees / (scale * 300) - (0.5 * scale)
}

fun Double.toDegrees() = this * 180 / PI
fun Double.toRadians() = this * PI / 180

fun powerCurve(value: Double) =
    if (value > 0) value.pow(2) // positive
    else -value.pow(2) // negative
