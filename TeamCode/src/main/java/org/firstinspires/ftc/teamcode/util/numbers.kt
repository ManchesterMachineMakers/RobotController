package org.firstinspires.ftc.teamcode.util

fun Double.equalsTolerance(other: Double, tolerance: Double)
    = (other - tolerance < this && this < other + tolerance)