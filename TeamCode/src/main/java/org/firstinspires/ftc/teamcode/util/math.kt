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
    val servoPosition = degrees / 300 * abs(scaleRange.second - scaleRange.first)
    return limitValue(servoPosition, 0.0, 1.0)
}

fun toDegrees(radians: Double) = radians * PI / 180

fun toRadians(degrees: Double) = degrees * 180 / PI

fun powerCurve(value: Double) =
    if (value > 0) value.pow(2) // positive
    else -value.pow(2) // negative

fun <T: Number> limitValue(value: T, min: T, max: T) =
    when {
        value.toDouble() > max.toDouble() -> max
        value.toDouble() < min.toDouble() -> min
        else -> value
    }
