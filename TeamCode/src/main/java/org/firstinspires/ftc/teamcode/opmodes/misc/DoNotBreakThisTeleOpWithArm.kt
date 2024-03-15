package org.firstinspires.ftc.teamcode.opmodes.misc

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.subassemblies.misc.DoNotBreakThisArm
import kotlin.math.*

@TeleOp(name = "Do Not Break This TeleOp with Arm", group = "do not break")
class DoNotBreakThisTeleOpWithArm : LinearOpMode() {
    override fun runOpMode() {

        val arm = DoNotBreakThisArm(this)

        val leftFront = hardwareMap.dcMotor.get("left_front")
        val rightFront = hardwareMap.dcMotor.get("right_front")
        val leftRear = hardwareMap.dcMotor.get("left_rear")
        val rightRear = hardwareMap.dcMotor.get("right_rear")


        leftFront.configure(DcMotorSimple.Direction.FORWARD)
        rightFront.configure(DcMotorSimple.Direction.REVERSE)
        leftRear.configure(DcMotorSimple.Direction.REVERSE)
        rightRear.configure(DcMotorSimple.Direction.FORWARD)

        waitForStart()

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
                val leftX = -gamepad1.left_stick_x.toDouble()
                val leftY = gamepad1.left_stick_y.toDouble()
                val rightX = -gamepad1.right_stick_x.toDouble()

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                val denominator = max(abs(leftY) + abs(leftX) + abs(rightX), 1.0)
                val leftFrontPower = (leftY + leftX + rightX) / denominator
                val rightFrontPower = (leftY - leftX - rightX) / denominator
                val leftRearPower = (leftY - leftX + rightX) / denominator
                val rightRearPower = (leftY + leftX - rightX) / denominator

                leftFront.power = leftFrontPower
                rightFront.power = rightFrontPower
                leftRear.power = leftRearPower
                rightRear.power = rightRearPower

                arm.loop()
                arm.telemetry()
                telemetry.update()
            }
        }
    }

    private fun DcMotor.configure(direction: DcMotorSimple.Direction) {
        this.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = direction
    }
}