package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.util.GamepadManager

@TeleOp(name = "Do Not Break This TeleOp with Arm")
class DoNotBreakThisTeleOpArm : LinearOpMode() {
    override fun runOpMode() {
        val leftFront = hardwareMap.dcMotor["left_front"]
        val rightFront = hardwareMap.dcMotor["right_front"]
        val leftRear = hardwareMap.dcMotor["left_rear"]
        val rightRear = hardwareMap.dcMotor["right_rear"]
        val motors = arrayOf(leftFront, rightFront, leftRear, rightRear)
        val arm = Arm(this)

        for (motor in motors) {
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        for (motor in arrayOf(leftRear, rightRear)) {
            motor.direction = DcMotorSimple.Direction.REVERSE // compensate for rear motors
        }
        for (motor in arrayOf(leftFront, rightFront)) {
            motor.direction = DcMotorSimple.Direction.FORWARD // front motors are normal
        }
        waitForStart()
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                val r =
                    Math.hypot(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble())
                val robotAngle = Math.atan2(
                    -gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble()
                ) - Math.PI / 4
                val rightX = gamepad1.right_stick_x
                val v1 = r * Math.cos(robotAngle) + rightX
                val v2 = r * Math.sin(robotAngle) - rightX
                val v3 = r * Math.sin(robotAngle) + rightX
                val v4 = r * Math.cos(robotAngle) - rightX
                leftFront.power = v1 / 1.2
                rightFront.power = v2 / 1.2
                leftRear.power = v3 / 1.2
                rightRear.power = v4 / 1.2
                arm.controller(GamepadManager(gamepad2))
            }
        }
    }
}