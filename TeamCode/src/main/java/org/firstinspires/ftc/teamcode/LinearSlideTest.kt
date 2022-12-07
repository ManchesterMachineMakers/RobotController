package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.RobotLog

@TeleOp(name = "Linear Slide Test")
class LinearSlideTest : LinearOpMode() {
    fun log(position: String) {
        telemetry.addLine("Running to " + position)
        telemetry.update()
        RobotLog.i("Running to " + position)
    }
    override fun runOpMode() {
        RobotConfig.initHardwareMaps(hardwareMap, gamepad1, gamepad2)
        RobotConfig.init()
        waitForStart();
        if(opModeIsActive()) {
            log("base")
            LinearSlide.goToBase()
            while(LinearSlide.isBusy());
            log("cone")
            LinearSlide.goToCone()
            while(LinearSlide.isBusy());
            log("low")
            LinearSlide.goToLow()
            while(LinearSlide.isBusy());
            log("mid")
            LinearSlide.goToMid()
            while(LinearSlide.isBusy());
            log("base")
            LinearSlide.goToBase()
            while(LinearSlide.isBusy());
        }
    }
}