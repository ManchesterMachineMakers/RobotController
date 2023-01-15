package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

public const val POWER_COEFFICIENT = 1.2

@TeleOp(name = "Basic TeleOp")
class BasicTeleOp : LinearOpMode() {
    override fun runOpMode() {
        RobotConfig.initHardwareMaps(hardwareMap, gamepad1, gamepad2)
        DriveBase.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        DriveBase.setTravelDirection(DriveBase.TravelDirection.base)
        telemetry.addLine("Connected hardware: ${RobotConfig.allConnected().map { it.javaClass.simpleName }.joinToString(", ")}")
        telemetry.update()
        waitForStart()
        if(opModeIsActive()) {
            if(Arm.exists()) Arm.zero()
            val directions = telemetry.addData("Directions", "")
            while(opModeIsActive()) {
                if(DriveBase.exists()) DriveBase.controller()
                if(Arm.exists()) Arm.controller()
                if(LinearSlide.exists()) LinearSlide.controller()
            }
        }
    }
}
