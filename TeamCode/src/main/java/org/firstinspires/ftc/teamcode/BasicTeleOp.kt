package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

public const val POWER_COEFFICIENT = 1.2

@TeleOp(name = "Basic TeleOp")
class BasicTeleOp : LinearOpMode() {
    override fun runOpMode() {
        RobotConfig.initHardwareMaps(hardwareMap, gamepad1, gamepad2)
        Arm.initHardware()
        RobotConfig.init()
        waitForStart()
        if(opModeIsActive()) {
            Arm.zero()
            while(opModeIsActive()) {
                val r = hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y)
                val robotAngle = atan2(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
                ) - Math.PI / 4
                val rightX = gamepad1.right_stick_x
                val v1 = r * cos(robotAngle) + rightX
                val v2 = r * sin(robotAngle) - rightX
                val v3 = r * sin(robotAngle) + rightX
                val v4 = r * cos(robotAngle) - rightX

                DriveBase.go(doubleArrayOf(-(v1 / POWER_COEFFICIENT), v2 / POWER_COEFFICIENT, -(v3 / POWER_COEFFICIENT), v4 / POWER_COEFFICIENT))

                Arm.controller()
            }
        }
    }
}
