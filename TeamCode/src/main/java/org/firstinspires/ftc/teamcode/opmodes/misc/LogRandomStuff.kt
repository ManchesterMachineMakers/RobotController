package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog

@TeleOp(name = "Log Random Stuff")
class LogRandomStuff : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addLine("Please watch the log.")
        telemetry.update()
        RobotLog.i("Logging during initialization")
        waitForStart()
        if(opModeIsActive()) {
            RobotLog.i("Logging during runtime")
        }
    }
}