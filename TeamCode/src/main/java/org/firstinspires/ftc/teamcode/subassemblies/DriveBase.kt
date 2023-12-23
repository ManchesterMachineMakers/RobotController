package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.util.GamepadManager

class DriveBase(opMode: OpMode) : Controllable, Subject {
    val hardwareMap = opMode.hardwareMap
    val leftFront = hardwareMap.dcMotor.get("leftFront")
    val rightFront = hardwareMap.dcMotor.get("rightFront")
    val leftRear = hardwareMap.dcMotor.get("leftRear")
    val rightRear = hardwareMap.dcMotor.get("rightRear")

    init {
        configMotor(leftFront, DcMotorSimple.Direction.FORWARD)
        configMotor(rightFront, DcMotorSimple.Direction.REVERSE)
        configMotor(leftRear, DcMotorSimple.Direction.FORWARD)
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE)
    }

    private fun configMotor(motor: DcMotor, direction: DcMotorSimple.Direction) {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER // Runs motor without distance tracking
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE // Brakes motor when stopping
        motor.direction = direction // Sets motors to their specified direction
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
        var leftBackPower = x + y - yaw
        var rightBackPower = x - y + yaw

        // Normalize wheel powers to be less than 1.0
        var max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower))
        max = Math.max(max, Math.abs(leftBackPower))
        max = Math.max(max, Math.abs(rightBackPower))
        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }

        // Send powers to the wheels.
        leftFront!!.power = leftFrontPower
        rightFront!!.power = rightFrontPower
        leftRear!!.power = leftBackPower
        rightRear!!.power = rightBackPower
    }

    override fun controller(gamepad: GamepadManager) {
        val r = Math.hypot(gamepad.gamepad.left_stick_x.toDouble(), -gamepad.gamepad.left_stick_y.toDouble())
        val robotAngle = Math.atan2(-gamepad.gamepad.left_stick_y.toDouble(), gamepad.gamepad.left_stick_x.toDouble()) - Math.PI / 4
        val rightX = gamepad.gamepad.right_stick_x.toDouble()
        val v1 = r * Math.cos(robotAngle) + rightX
        val v2 = r * Math.sin(robotAngle) - rightX
        val v3 = r * Math.sin(robotAngle) + rightX
        val v4 = r * Math.cos(robotAngle) - rightX
        leftFront!!.power = v1 / 1.2
        rightFront!!.power = v2 / 1.2
        leftRear!!.power = v3 / 1.2
        rightRear!!.power = v4 / 1.2
    }
}
