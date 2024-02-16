package org.firstinspires.ftc.teamcode.util

import kotlin.math.*

fun clamp(x: Double, lowerBound: Double, upperBound: Double)
    = max(lowerBound, min(upperBound, x))

fun equalsTolerance(a: Double, b: Double, tolerance: Double)
    = abs(b - a) < tolerance

fun encoderPositionToDegrees(encoderPosition: Int, encoderResolution: Double)
    = 360 * encoderPosition / encoderResolution

/** Assumes use of a 300 degree non-continuous servo */
fun degreesToServoPosition(degrees: Double, scaleRange: Pair<Double, Double>) =
    degrees / 300 * abs(scaleRange.second - scaleRange.first)

fun toDegrees(radians: Double) = radians * PI / 180

fun toRadians(degrees: Double) = degrees * 180 / PI

fun powerCurve(stickValue: Double) =
    if (stickValue > 0) stickValue.pow(2) // positive
    else -stickValue.pow(2) // negative