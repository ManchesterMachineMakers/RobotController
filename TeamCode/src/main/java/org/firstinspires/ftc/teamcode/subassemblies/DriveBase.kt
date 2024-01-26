package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.config
import kotlin.math.*

class DriveBase(opMode: OpMode, gamepad: Gamepad) : Subassembly(opMode, gamepad, "Drive Base") {

    private val leftFront: DcMotor = hardwareMap.dcMotor.get("left_front")
    private val rightFront: DcMotor = hardwareMap.dcMotor.get("right_front")
    private val leftRear: DcMotor = hardwareMap.dcMotor.get("left_rear")
    private val rightRear: DcMotor = hardwareMap.dcMotor.get("right_rear")

    init {
        leftFront.config(DcMotorSimple.Direction.REVERSE)
        rightFront.config(DcMotorSimple.Direction.FORWARD)
        leftRear.config(DcMotorSimple.Direction.FORWARD)
        rightRear.config(DcMotorSimple.Direction.REVERSE)
    }

    override fun loop() {
        loopTime.reset()

        val r = hypot(gamepad.left_stick_x.toDouble(), -gamepad.left_stick_y.toDouble())
        val robotAngle = atan2(-gamepad.left_stick_y.toDouble(), gamepad.left_stick_x.toDouble()) - PI / 4
        val rightX = gamepad.right_stick_x.toDouble()

        val v1 = r * sin(robotAngle) + rightX
        val v2 = r * cos(robotAngle) - rightX
        val v3 = r * cos(robotAngle) + rightX
        val v4 = r * sin(robotAngle) - rightX

        leftFront.power = curveDouble(v1) * FRONT_POWER
        rightFront.power = curveDouble(v2) * FRONT_POWER
        leftRear.power = curveDouble(v3) * REAR_POWER
        rightRear.power = curveDouble(v4) * REAR_POWER
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addLine()
    }

    /**
     * Taken from the RobotAutoDriveToAprilTagOmni example (2023) Move robot according to desired
     * axes motions
     *
     * Positive X is forward
     *
     * Positive Y is strafe left
     *
     * Positive Yaw is
     * counter-clockwise
     */
    fun moveRobot(x: Double, y: Double, yaw: Double) {
        // Calculate wheel powers.
        var leftFrontPower = x - y - yaw
        var rightFrontPower = x + y + yaw
        var leftBackPower =  x + y - yaw
        var rightBackPower =  x - y + yaw

        // Normalize wheel powers to be less than 1.0
        var max = max(abs(leftFrontPower), abs(rightFrontPower))
        max = max(max, abs(leftBackPower))
        max = max(max, abs(rightBackPower))
        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }

        // Send powers to the wheels.
        leftFront.power = leftFrontPower
        rightFront.power = rightFrontPower
        leftRear.power = leftBackPower
        rightRear.power = rightBackPower
    }

    private fun curveDouble(num: Double): Double {
        return  if (num > 0) num.pow(2)
                else -num.pow(2)
    }

    companion object {
        const val FRONT_POWER = 0.8
        const val REAR_POWER = 0.8
    }
}
