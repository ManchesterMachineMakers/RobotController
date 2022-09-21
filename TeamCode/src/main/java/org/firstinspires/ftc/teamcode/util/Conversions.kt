package org.firstinspires.ftc.teamcode.util

object Conversions {
    fun mmToInches(mm: Double): Double {
        return mm / mmPerInch
    }

    fun inchesToMm(inches: Double): Double {
        return inches * mmPerInch
    }

    val mmPerInch = 25.4f
}