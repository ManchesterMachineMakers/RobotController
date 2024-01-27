package org.firstinspires.ftc.teamcode.util

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

fun clamp(x: Double, lowerBound: Double, upperBound: Double)
    = max(lowerBound, min(upperBound, x))

fun equalsTolerance(a: Double, b: Double, tolerance: Double)
    = abs(b - a) < tolerance