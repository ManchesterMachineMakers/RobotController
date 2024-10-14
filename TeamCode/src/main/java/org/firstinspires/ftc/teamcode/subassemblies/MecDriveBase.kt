package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.powerCurve
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.roundToInt

class MecDriveBase(opMode: LinearOpMode) : Subassembly(opMode, "Mecanum Drive Base") {

    private val leftFront = hardwareMap.dcMotor.get("left_front")
    private val rightFront = hardwareMap.dcMotor.get("right_front")
    private val leftRear = hardwareMap.dcMotor.get("left_rear")
    private val rightRear = hardwareMap.dcMotor.get("right_rear")

    init {
        // direction = FORWARD by default
        //leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.REVERSE
        leftRear.direction = DcMotorSimple.Direction.REVERSE
        //rightRear.direction = DcMotorSimple.Direction.REVERSE

        opMode.log("DriveBase successfully initialized")
    }

    fun control(gamepad: Gamepad) {
        // from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        val leftX: Double = -gamepad.left_stick_x.toDouble()
        val leftY: Double = gamepad.left_stick_y.toDouble()
        val rightX: Double = -gamepad.right_stick_x.toDouble()

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

        leftFront.power = powerCurve(leftFrontPower)
        rightFront.power = powerCurve(rightFrontPower)
        leftRear.power = powerCurve(leftRearPower)
        rightRear.power = powerCurve(rightRearPower)
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addLine()
    }
}