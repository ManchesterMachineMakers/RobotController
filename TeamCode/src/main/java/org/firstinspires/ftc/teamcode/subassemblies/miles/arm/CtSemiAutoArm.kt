package org.firstinspires.ftc.teamcode.subassemblies.miles.arm

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.util.BaseArm
import kotlin.math.*

/**
 * Semi-automatic arm subassembly for controlling arm and wrist movements.
 */
class CtSemiAutoArm(opMode: OpMode) : BaseArm(opMode) {
    
    override val name = "Continuous Semi-Auto Arm"
    
    private var theta = 60.0 // degrees
    private var wristAlignment = "easel"

    /**
     * Main loop for controlling the semi-auto arm.
     */
    override fun loop() {
        loopTime.reset()

        // Arm movement
        if (!arm.isOverCurrent) {
            if (gamepad.left_stick_y != 0f) {
                arm.mode = DcMotor.RunMode.RUN_USING_ENCODER
                arm.velocity = gamepad.left_stick_y * ARM_ENCODER_RES * ARM_SPEED
                latestArmPosition = arm.currentPosition
            } else {
                brakeArm()
            }
        }

        // Wrist movement
        wrist.position = findWristPosition()

        // Reset arm position
        if (gamepad.b) {
            arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }
        // Wrist alignment
        if (gamepad.a) {
            theta = 120.0
            wristAlignment = "floor"
        } else if (gamepad.y) {
            theta = 60.0
            wristAlignment = "easel"
        }
        handlePixelDroppers()
        handleOvercurrentProtection()
        handleAirplaneLauncher()
    }

    /**
     * Displays relevant telemetry information.
     */
    override fun telemetry() {

        // Telemetry updates
        telemetry.addData("Semi-Automatic Arm", "")
        telemetry.addData("status", currentStatus)
        telemetry.addData("run time (seconds)", opMode.runtime.toInt())
        telemetry.addData("loop time (milliseconds)", loopTime.milliseconds().toInt())
        telemetry.addData("arm mode", arm.mode)
        telemetry.addData("arm velocity", arm.velocity)
        telemetry.addData("wrist alignment", wristAlignment)
        telemetry.addData("arm motor current (amps)", arm.getCurrent(CurrentUnit.AMPS))
        telemetry.addLine()
    }

    /**
     * Calculates the wrist position based on arm angle and theta.
     *
     * @return The calculated wrist position.
     */
    private fun findWristPosition(): Double {
        val armAngle = 360 * arm.currentPosition / ARM_ENCODER_RES
        val servoAngle = 90 + theta - GAMMA - armAngle
        return (servoAngle - 90) / (0.53 * 300) - 0.5 * 0.53
    }

    companion object {
        private val GAMMA = atan(16.0 / 283.0) * (180 / PI) // for math
    }
}