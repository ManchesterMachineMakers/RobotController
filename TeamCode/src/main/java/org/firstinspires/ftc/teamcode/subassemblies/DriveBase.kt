package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.manchestermachinemakers.easyop.Device
import org.manchestermachinemakers.easyop.Subassembly

class DriveBase : Subassembly, Controllable {
    @Device lateinit var leftFront: DcMotor
    @Device lateinit var rightFront: DcMotor
    @Device lateinit var leftRear: DcMotor
    @Device lateinit var rightRear: DcMotor

    /**
     * Taken from the RobotAutoDriveToAprilTagOmni example (2023) Move robot according to desired
     * axes motions <p> Positive X is forward <p> Positive Y is strafe left <p> Positive Yaw is
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
        leftFront.setPower(leftFrontPower)
        rightFront.setPower(rightFrontPower)
        leftRear.setPower(leftBackPower)
        rightRear.setPower(rightBackPower)
    }

    override fun controller(gamepad: GamepadManager) {
        val r = hypot(gamepad.gamepad.left_stick_x, -gamepad.gamepad.left_stick_y)
        val robotAngle =
                atan2(-gamepad.gamepad.left_stick_y, gamepad.gamepad.left_stick_x) - Math.PI / 4
        val rightX = gamepad.gamepad.right_stick_x
        val v1 = r * cos(robotAngle) + rightX
        val v2 = r * sin(robotAngle) - rightX
        val v3 = r * sin(robotAngle) + rightX
        val v4 = r * cos(robotAngle) - rightX
        leftFront.setPower(v1 / 1.2)
        rightFront.setPower(v2 / 1.2)
        leftRear.setPower(v3 / 1.2)
        rightRear.setPower(v4 / 1.2)
    }
}
