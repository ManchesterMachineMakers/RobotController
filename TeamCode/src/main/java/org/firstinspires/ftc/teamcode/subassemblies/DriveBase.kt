package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.autonomous.path.strafingCoefficient
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.config
import kotlin.math.*

class DriveBase(opMode: OpMode) : Subassembly(opMode, "Drive Base") {

    private val leftFront: DcMotor = hardwareMap.dcMotor.get("left_front")
    private val rightFront: DcMotor = hardwareMap.dcMotor.get("right_front")
    private val leftRear: DcMotor = hardwareMap.dcMotor.get("left_rear")
    private val rightRear: DcMotor = hardwareMap.dcMotor.get("right_rear")
    companion object {
        const val strafeCoeff = 0.5
        const val FRONT_POWER = 0.6
        const val REAR_POWER = 0.8
    }

    init {
        leftFront.config(DcMotorSimple.Direction.REVERSE)
        rightFront.config(DcMotorSimple.Direction.FORWARD)
        leftRear.config(DcMotorSimple.Direction.FORWARD)
        rightRear.config(DcMotorSimple.Direction.REVERSE)
    }

    override fun loop(gamepad: Gamepad) {
        loopTime.reset()

        val r = hypot(gamepad.left_stick_x.toDouble(), -gamepad.left_stick_y.toDouble())
        val robotAngle = atan2(-gamepad.left_stick_y.toDouble(), gamepad.left_stick_x.toDouble()) - PI / 4
        val rightX = gamepad.right_stick_x.toDouble()

        val v1 = r * sin(robotAngle) + rightX
        val v2 = r * cos(robotAngle) - rightX
        val v3 = r * cos(robotAngle) + rightX
        val v4 = r * sin(robotAngle) - rightX

        leftFront.power = curveDouble(v1) / FRONT_POWER
        rightFront.power = curveDouble(v2) / FRONT_POWER
        leftRear.power = curveDouble(v3) / REAR_POWER
        rightRear.power = curveDouble(v4) / REAR_POWER
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addLine()
    }

    data class MoveRobotCalculations(
            val x: Double,
            val y: Double,
            val yaw: Double,

            val leftFront: Double = x - y - yaw,
            val rightFront: Double = x + y + yaw,
            val leftRear: Double = strafeCoeff * (x + y - yaw),
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

    fun controller(gamepad: GamepadManager) {
        val r = Math.hypot(
            gamepad.gamepad.left_stick_x.toDouble(),
            -gamepad.gamepad.left_stick_y.toDouble()
        )
        val robotAngle = Math.atan2(
            -gamepad.gamepad.left_stick_y.toDouble(),
            gamepad.gamepad.left_stick_x.toDouble()
        ) - Math.PI / 4
        val rightX = gamepad.gamepad.right_stick_x.toDouble()
        val v1 = r * Math.cos(robotAngle) + rightX
        val v2 = r * Math.sin(robotAngle) - rightX
        val v3 = r * Math.sin(robotAngle) + rightX
        val v4 = r * Math.cos(robotAngle) - rightX
        leftFront.power = v1 / 1.2
        rightFront.power = v2 / 1.2
        leftRear.power = v3 / 1.2
        rightRear.power = v4 / 1.2
    }

    private fun curveDouble(num: Double): Double {
        return  if (num > 0) num.pow(2)
                else -num.pow(2)
    }
}
