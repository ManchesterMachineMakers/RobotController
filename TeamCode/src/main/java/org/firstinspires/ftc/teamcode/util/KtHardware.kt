package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

object KtHardware {
    inline fun <reified T : Subassembly> get(om: LinearOpMode): T? = RobotConfig.CURRENT.getHardware(T::class.java, om)
    fun name(n: String): String = RobotConfig.CURRENT.name(n)
}