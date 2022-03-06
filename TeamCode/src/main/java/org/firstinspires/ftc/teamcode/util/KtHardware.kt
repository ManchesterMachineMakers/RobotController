package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode

object KtHardware {
    inline fun <reified T : Subassembly> get(om: OpMode): T = RobotConfig.CURRENT.getHardware(T::class.java, om)
    fun name(n: String): String = RobotConfig.CURRENT.name(n)
}