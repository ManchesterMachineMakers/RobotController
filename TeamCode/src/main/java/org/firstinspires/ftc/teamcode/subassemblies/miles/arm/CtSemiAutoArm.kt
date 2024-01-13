package org.firstinspires.ftc.teamcode.subassemblies.miles.arm

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.bases.BaseArm
import kotlin.math.*

/**
 * Semi-automatic arm subassembly for controlling arm and wrist movements.
 */
class CtSemiAutoArm(opMode: OpMode, gamepad: Gamepad) : BaseArm(opMode, gamepad, "Continuous Semi-Auto Arm") {
    private var theta = 60 // degrees
    /** Position of the wrist to stay level with the floor or easel. */
    private val relativeWristPosition: Double
        get() {
            val armAngle = arm.currentPosition / 2 // TODO: Figure out what is going on with this
            val servoAngle = 90 + theta - GAMMA - armAngle // degrees?
            return (servoAngle - 90) / (0.53 * 300) - 0.5 * 0.53 // from degrees? to servo range
        }

    /**
     * Main loop for controlling the semi-auto arm.
     */
    override fun loop() {
        loopTime.reset()

        // Arm movement
        if (!arm.isOverCurrent) {
            arm.mode = DcMotor.RunMode.RUN_USING_ENCODER
            // Power curve
            val leftY = gamepad.left_stick_y
            arm.power =
                if (leftY > 0.0) leftY.pow(2) * ARM_POWER
                else -leftY.pow(2) * ARM_POWER
        }

        // Wrist movement
        wrist.position = relativeWristPosition

        // Reset arm position
        if (gamepad.b) arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Wrist alignment
        if (gamepad.a) theta = 120
        else if (gamepad.y) theta = 60

        handleAllRobotBits()
    }

    /**
     * Displays relevant telemetry information.
     */
    override fun telemetry() {
        val intakeAlignment =
            when (theta) {
                120 -> "floor"
                60 -> "easel"
                else -> "unknown"
            }

        // Telemetry updates
        super.telemetry()
        telemetry.addData("theta", theta)
        telemetry.addData("intake alignment", intakeAlignment)
        telemetry.addData("target wrist position", relativeWristPosition)
        telemetry.addData("actual wrist position", wrist.position)
        telemetry.addLine()
    }

    companion object {
        private val GAMMA = atan(16.0 / 283.0) * (180 / PI) // for math
    }
}