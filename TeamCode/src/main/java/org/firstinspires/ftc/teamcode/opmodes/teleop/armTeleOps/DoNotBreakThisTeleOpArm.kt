package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.DoNotBreakThisArm
import kotlin.math.*

@TeleOp(name = "Do Not Break This TeleOp with Arm", group = "do not break")
class DoNotBreakThisTeleOpArm : LinearOpMode() {
    override fun runOpMode() {

        val arm = DoNotBreakThisArm(this)

        val leftFront = hardwareMap.dcMotor.get("left_front")
        val rightFront = hardwareMap.dcMotor.get("right_front")
        val leftRear = hardwareMap.dcMotor.get("left_rear")
        val rightRear = hardwareMap.dcMotor.get("right_rear")

        leftFront.configure(DcMotorSimple.Direction.REVERSE)
        rightFront.configure(DcMotorSimple.Direction.FORWARD)
        leftRear.configure(DcMotorSimple.Direction.FORWARD)
        rightRear.configure(DcMotorSimple.Direction.REVERSE)

        waitForStart()
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                val r = hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y)
                val robotAngle = atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - PI / 4
                val rightX = gamepad1.right_stick_x

                val v1 = r * cos(robotAngle) + rightX
                val v2 = r * sin(robotAngle) - rightX
                val v3 = r * sin(robotAngle) + rightX
                val v4 = r * cos(robotAngle) - rightX

                leftFront.power = v1 / 1.2
                rightFront.power = v2 / 1.2
                leftRear.power = v3 / 1.2
                rightRear.power = v4 / 1.2

                arm.loop()

                arm.telemetry()
            }
        }
    }

    private fun DcMotor.configure(direction: DcMotorSimple.Direction) {
        this.mode = DcMotor.RunMode.RUN_USING_ENCODER
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = direction
    }
}