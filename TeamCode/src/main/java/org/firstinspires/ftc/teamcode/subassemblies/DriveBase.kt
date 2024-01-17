package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.util.GamepadManager
import kotlin.math.abs

class DriveBase(opMode: OpMode) : Controllable, Subject {
    val hardwareMap = opMode.hardwareMap
    val leftFront = hardwareMap.dcMotor.get("left_front")
    val rightFront = hardwareMap.dcMotor.get("right_front")
    val leftRear = hardwareMap.dcMotor.get("left_rear")
    val rightRear = hardwareMap.dcMotor.get("right_rear")

    init {
        configMotor(leftFront, DcMotorSimple.Direction.REVERSE)
        configMotor(rightFront, DcMotorSimple.Direction.FORWARD)
        configMotor(leftRear, DcMotorSimple.Direction.FORWARD)
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE)
    }

    private fun configMotor(motor: DcMotor, direction: DcMotorSimple.Direction) {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER // Runs motor without distance tracking
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE // Brakes motor when stopping
        motor.direction = direction // Sets motors to their specified direction
    }

    data class MoveRobotCalculations(
            val x: Double,
            val y: Double,
            val yaw: Double,

            val leftFront: Double = x - y - yaw,
            val rightFront: Double = x + y + yaw,
            val leftRear: Double = x + y - yaw,
            val rightRear: Double = x - y + yaw
    )

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
        var (_, _, _,
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower) = MoveRobotCalculations(x, y, yaw)

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

    val motors = listOf(leftFront, rightFront, leftRear, rightRear)
    fun List<DcMotor>.setMode(mode: DcMotor.RunMode) {
        map { it.mode = mode }
    }

    fun setMode(mode: RunMode) {
        motors.setMode(mode)
    }

    fun setTargetPositions(lf: Int, rf: Int, lr: Int, rr: Int) {
        leftFront.targetPosition = lf
        rightFront.targetPosition = rf
        leftRear.targetPosition = lr
        rightRear.targetPosition = rr
    }

    fun runToPosition(lf: Int, rf: Int, lr: Int, rr: Int) {
        setMode(RunMode.STOP_AND_RESET_ENCODER)
        setMode(RunMode.RUN_USING_ENCODER)
        setTargetPositions(abs(lf), abs(rf), abs(lr), abs(rr))
        motors.map { (it as DcMotorEx).targetPositionTolerance = 5 }
        setMode(RunMode.RUN_TO_POSITION)

        leftFront.power = if (lf < 0) -0.5 else 0.5
        rightFront.power = if (rf < 0) -0.5 else 0.5
        leftRear.power = if (lr < 0) -0.5 else 0.5
        rightRear.power = if (rr < 0) -0.5 else 0.5
    }

    fun runToPosition(ticks: Int) {
        runToPosition(ticks, ticks, ticks, ticks)
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
