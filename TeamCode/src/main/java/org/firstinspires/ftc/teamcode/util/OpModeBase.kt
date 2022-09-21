package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.configurations.programmingBoard

abstract class OpModeBase : LinearOpMode() {
    override fun runOpMode() {
        RobotConfig.activate(hardwareMap, ::programmingBoard)
        run()
    }

    abstract fun run()
}