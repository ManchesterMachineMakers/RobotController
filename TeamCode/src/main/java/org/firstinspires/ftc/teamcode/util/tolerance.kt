package org.firstinspires.ftc.teamcode.util
import kotlin.math.abs

fun isWithinTolerance(a: Double, b: Double, tolerance: Double) = abs(b - a) < abs(tolerance)