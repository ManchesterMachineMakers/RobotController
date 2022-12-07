package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.RobotLog

@TeleOp(name = "Linear Slide Test")
class LinearSlideTest : LinearOpMode() {
    override fun runOpMode() {
        RobotConfig.initHardwareMaps(hardwareMap, gamepad1, gamepad2)
        RobotConfig.init()
        waitForStart();
        if(opModeIsActive()) {
            telemetry.addLine("Running to top")
            telemetry.update()
            RobotLog.i("Running to top")
            LinearSlide.goToTop()
            while(LinearSlide.isBusy());
            telemetry.addLine("Running to bottom")
            telemetry.update()
            RobotLog.i("Running to bottom")
            LinearSlide.goToBottom()
            while(LinearSlide.isBusy());
        }
    }
}