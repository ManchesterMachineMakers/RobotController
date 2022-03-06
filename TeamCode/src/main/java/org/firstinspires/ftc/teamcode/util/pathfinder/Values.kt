package org.firstinspires.ftc.teamcode.util.pathfinder

import kotlin.math.round

object Values {
    val ticksPerMM = 2.3663 // NOTE:  determined this number emperically by running a set # of ticks and dividing by the actual distance traveled
    fun getTicks(mm: Double) = round(ticksPerMM * mm).toInt()
}