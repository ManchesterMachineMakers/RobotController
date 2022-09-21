package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

// DO NOT REMOVE
import org.firstinspires.ftc.teamcode.util.configurations.fullRobot
import org.firstinspires.ftc.teamcode.util.configurations.programmingBoard

abstract class OpModeBase : LinearOpMode() {
    override fun runOpMode() {
        RobotConfig.activate(hardwareMap, ::fullRobot)
        run()
    }

    abstract fun run()
}