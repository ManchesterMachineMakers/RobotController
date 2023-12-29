package org.firstinspires.ftc.teamcode.opmodes.misc

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.config
import kotlin.math.*

@TeleOp(name = "Jerry Was A Race Car Driver")
class RaceCar : OpMode() {

    private val leftFront = hardwareMap.dcMotor.get("left_front")
    private val rightFront = hardwareMap.dcMotor.get("right_front")
    private val leftRear = hardwareMap.dcMotor.get("left_rear")
    private val rightRear = hardwareMap.dcMotor.get("right_rear")

    override fun init() {
        leftFront.config(DcMotorSimple.Direction.FORWARD)
        rightFront.config(DcMotorSimple.Direction.FORWARD)
        leftRear.config(DcMotorSimple.Direction.REVERSE)
        rightRear.config(DcMotorSimple.Direction.REVERSE)
    }

    override fun loop() {
        val r = hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y)
        val angleOfRacing = atan2(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x
        ) - Math.PI / 4

        val rightX = gamepad1.right_stick_x

        val v1 = r * cos(angleOfRacing) + rightX
        val v2 = r * sin(angleOfRacing) - rightX
        val v3 = r * sin(angleOfRacing) + rightX
        val v4 = r * cos(angleOfRacing) - rightX

        leftFront.power = v1 / 1.2
        rightFront.power = v2 / 1.2
        leftRear.power = v3 / 1.2
        rightRear.power = v4 / 1.2
    }
}