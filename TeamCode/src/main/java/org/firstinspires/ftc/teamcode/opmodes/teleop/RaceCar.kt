package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.manchestermachinemakers.easyop.Linear
import kotlin.math.*

@TeleOp(name = "Jerry Was A Race Car Driver")
class RaceCar : Linear() {

    private val leftFront = hardwareMap.dcMotor.get("left_front")
    private val rightFront = hardwareMap.dcMotor.get("right_front")
    private val leftRear = hardwareMap.dcMotor.get("left_rear")
    private val rightRear = hardwareMap.dcMotor.get("right_rear")

    override fun opInit() {
        leftFront.config(  DcMotorSimple.Direction.REVERSE )
        rightFront.config( DcMotorSimple.Direction.FORWARD )
        leftRear.config(   DcMotorSimple.Direction.FORWARD )
        rightRear.config(  DcMotorSimple.Direction.REVERSE )
    }

    override fun opLoop() {
        val r = hypot(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble())
        val robotAngle = atan2(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble()
        ) - Math.PI / 4

        val rightX = gamepad1.right_stick_x

        val v1 = r * cos(robotAngle) + rightX
        val v2 = r * sin(robotAngle) - rightX
        val v3 = r * sin(robotAngle) + rightX
        val v4 = r * cos(robotAngle) - rightX

        leftFront.power = v1 / 1.2
        rightFront.power = v2 / 1.2
        leftRear.power = v3 / 1.2
        rightRear.power = v4 / 1.2
    }

    private fun DcMotor.config(direction: DcMotorSimple.Direction) {
        this.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        this.direction = direction
    }
}