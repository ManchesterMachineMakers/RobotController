package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.teamcode.util.Subassembly
import kotlin.math.*

class DriveBase(opMode: OpMode, gamepad: Gamepad) : Subassembly(opMode, gamepad), Subject {

    override val name = "Drive Base"

    private val leftFront = hardwareMap.dcMotor.get("leftFront")!!
    private val rightFront = hardwareMap.dcMotor.get("rightFront")!!
    private val leftRear = hardwareMap.dcMotor.get("leftRear")!!
    private val rightRear = hardwareMap.dcMotor.get("rightRear")!!

    init {
        leftFront.config(DcMotorSimple.Direction.FORWARD)
        rightFront.config(DcMotorSimple.Direction.REVERSE)
        leftRear.config(DcMotorSimple.Direction.FORWARD)
        rightRear.config(DcMotorSimple.Direction.REVERSE)
    }

    override fun loop() {
        super.loop()

        val r = hypot(gamepad.left_stick_x.toDouble(), -gamepad.left_stick_y.toDouble())
        val robotAngle = atan2(-gamepad.left_stick_y.toDouble(), gamepad.left_stick_x.toDouble()) - PI / 4
        val rightX = gamepad.right_stick_x.toDouble()

        val v1 = r * cos(robotAngle) + rightX
        val v2 = r * sin(robotAngle) - rightX
        val v3 = r * sin(robotAngle) + rightX
        val v4 = r * cos(robotAngle) - rightX

        leftFront.power = v1 / 1.2
        rightFront.power = v2 / 1.2
        leftRear.power = v3 / 1.2
        rightRear.power = v4 / 1.2
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

    private fun DcMotor.config(direction: DcMotorSimple.Direction) {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER // Runs motor without distance tracking
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE // Brakes motor when stopping
        this.direction = direction // Sets motors to their specified direction
    }
}
