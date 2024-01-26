package org.firstinspires.ftc.teamcode.subassemblies.miles.arm

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.teamcode.util.bases.BaseArm
import kotlin.math.*

class IncSemiAutoArm(opMode: OpMode) : BaseArm(opMode, "Incremented Semi-Auto Arm") {
    /** The angle at which the intake will be level with. */
    private var theta = 60.0 // 60 for easel, 120 for floor
    private var pixelLayer = 0

    private val targetArmAndWristPosition: Pair<Int, Double>
        /**
         * See math: [...](https://drive.google.com/file/d/1ADeKl-3EPOc8nBHZwGThREwBQAEdIAJ9/view)
         * Calculates the target positions for arm and wrist.
         *
         * @return A map entry containing the target arm position and target wrist position.
         */
        get() {
            val argument = ((D1 + pixelLayer * D2 - L3) * sin(theta) + L2 * cos(theta) - H) / R
            var alpha = asin(argument)
            telemetry.addData("argument", argument)
            telemetry.addData("alpha", alpha)

            // Ensure alpha stays within limits
            if (alpha < -1) {
                alpha = -1.0
            } else if (alpha > 1) {
                alpha = 1.0
            }
            val beta =
                if (pixelLayer <= -1) 0.5
                else theta - GAMMA - alpha

            val targetArmPosition = (alpha / 360 * ARM_ENCODER_RES).toInt() // from degrees (alpha) to encoder ticks (targetArmPosition)
            val targetWristPosition = beta * 0.53 * 300 / 360 // from degrees (beta) to the servo's range (targetWristPosition) (53% of 300 degrees)

            return Pair(targetArmPosition, targetWristPosition)
        }

    /**
     * Main loop for controlling the semi-auto arm.
     */
    override fun loop(gamepad: Gamepad) {
        val gamepadManager = GamepadManager(gamepad)
        loopTime.reset()

        // Set target positions for arm and wrist
        arm.targetPosition = targetArmAndWristPosition.first
        wrist.position = targetArmAndWristPosition.second

        // Incrementer
        gamepadManager.once("dpad_up") { pixelLayer++ }
        gamepadManager.once("dpad_down") { pixelLayer-- }
        gamepadManager.once("dpad_right") { pixelLayer += ARM_LARGE_INCREMENT }
        gamepadManager.once("dpad_left") { pixelLayer -= ARM_LARGE_INCREMENT }

        // Ensure pixelLayer stays within limits
        if (pixelLayer < -1) pixelLayer = -1
        else if (pixelLayer > ARM_INCREMENT_UPPER_LIMIT) pixelLayer = ARM_INCREMENT_UPPER_LIMIT

        // Wrist alignment based on gamepad input
        if (gamepad.a) theta = 120.0
        else if (gamepad.y) theta = 60.0

        // Encoder reset
        arm.mode =
            if (gamepad.b) DcMotor.RunMode.STOP_AND_RESET_ENCODER
            else DcMotor.RunMode.RUN_TO_POSITION

        // Hook onto bar for winching
        if (gamepad.x) {
            wrist.position = WRIST_WINCH_POSITION
            arm.targetPosition = ARM_WINCH_POSITION
            arm.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        handleAllRobotBits(gamepad)
    }

    /**
     * Displays relevant telemetry information.
     */
    override fun telemetry() {
        // Telemetry updates
        super.telemetry()
        telemetry.addData("arm target position", arm.targetPosition)
        telemetry.addData("arm pixel layer", pixelLayer)
        telemetry.addData("theta", theta)
        telemetry.addLine()
    }

    companion object {
        // Constants for arm increments and limits
        private const val ARM_LARGE_INCREMENT = 4
        private const val ARM_INCREMENT_UPPER_LIMIT = 7

        // Position presets for arm and wrist
        private const val ARM_WINCH_POSITION = 0 // TODO: get value for this
        private const val WRIST_WINCH_POSITION = 0.0 // TODO: get value for this

        // Math constants for arm calculations
        private val GAMMA = atan(16.0 / 283.0) * (180 / PI)
        private const val D1 = 220.0
        private const val D2 = 88.9
        private const val L2 = 67.88
        private const val L3 = 59.5
        private const val H = 287.75
        private const val R = 336.0
    }
}