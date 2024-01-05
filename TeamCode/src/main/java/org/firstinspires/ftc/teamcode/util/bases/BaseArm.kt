package org.firstinspires.ftc.teamcode.util.bases

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.util.Subassembly
import kotlin.math.*

/**
 * Abstract class representing a basic arm subassembly for FTC robotics.
 */
abstract class BaseArm(opMode: OpMode, gamepad: Gamepad) : Subassembly(opMode, gamepad) {

    // Motors
    protected val arm: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "arm")
    protected val winch: DcMotor = hardwareMap.dcMotor.get("winch")

    // Servos
    protected val wrist: Servo = hardwareMap.servo.get("wrist")
    protected val airplaneLauncher: Servo = hardwareMap.servo.get("airplane_launcher")
    protected val leftRelease: Servo = hardwareMap.servo.get("left_release")
    protected val rightRelease: Servo = hardwareMap.servo.get("right_release")

    protected val intakeTouch: TouchSensor = hardwareMap.touchSensor.get("intake")

    // Arm state variables
    protected var latestArmPosition = 0 // in encoder ticks
    protected var latestWinchPosition = 0
    protected var buttonWasPressed = false
    protected var airplaneLauncherToggle = false // false = closed, true = open
    protected var allowWinchMovement = true

    init {
        status = "initializing"

        // Arm motor configuration
        arm.direction = DcMotorSimple.Direction.REVERSE
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS)


        // Wrist servo configuration
        wrist.scaleRange(0.25, 0.78) // 53% of 300-degree range
        wrist.direction = Servo.Direction.FORWARD

        // Arm servo configuration
        airplaneLauncher.scaleRange(0.0, 1.0) // 1 should be open, 0 should be closed; TODO: Get these values
        airplaneLauncher.direction = Servo.Direction.FORWARD // TODO: Get ideal direction

        // Left release servo configuration
        leftRelease.scaleRange(0.15, 0.40) // 22.5% of 300-degree range
        leftRelease.direction = Servo.Direction.FORWARD

        // Right release servo configuration
        rightRelease.scaleRange(0.6, 1.0) // 22.5% of 300-degree range
        rightRelease.direction = Servo.Direction.REVERSE

        // Initialize arm position and button state
        latestArmPosition = arm.currentPosition
        buttonWasPressed = false

        // Display initialization completion message
        telemetry.addData(">", "Arm Subassembly Ready")
    }


    // Runs once after start is pressed, before loop()
    open fun start() {
        airplaneLauncher.position = 0.0 // ensure launcher is closed
    }

    abstract override fun loop()

    override fun telemetry() {
        telemetry.addLine(name)
        telemetry.addData("status", status)
        telemetry.addData("run time (seconds)", opMode.runtime)
        telemetry.addData("loop time (milliseconds)", loopTime.milliseconds())
        telemetry.addData("arm mode", arm.mode)
        telemetry.addData("arm position", arm.currentPosition)
        telemetry.addData("arm current (amps)", arm.getCurrent(CurrentUnit.AMPS).toString() + " out of : " + ARM_OVERCURRENT_THRESHOLD)
    }

    /**
     * Check for overcurrent condition and take appropriate action.
     */
    protected fun handleOvercurrentProtection() {
        if (arm.isOverCurrent) {
            // Display warning message
            telemetry.addData("WARNING", "arm motor is overcurrent, reduce load or the arm may break")

            // Stop opMode if overcurrent is severe
            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                opMode.requestOpModeStop()
            } else {
                arm.targetPosition = 0
                arm.mode = DcMotor.RunMode.RUN_TO_POSITION
            }
        }
    }

    /**
     * Control the intake servos based on gamepad input.
     */
    protected fun handlePixelDroppers() {
        // Left release servo control
        if (gamepad.left_bumper) { // Open
            leftRelease.position = 1.0
        } else if (gamepad.left_trigger > 0.2) { // Close
            leftRelease.position = 0.0
        }
        // Right release servo control
        if (gamepad.right_bumper) { // Open
            rightRelease.position = 1.0
        } else if (gamepad.right_trigger > 0.2) { // Close
            rightRelease.position = 0.0
        }
    }

    protected fun handleAirplaneLauncher() {
        // Airplane launcher
        if (gamepad.x) {
            airplaneLauncherToggle = !airplaneLauncherToggle
            if (airplaneLauncherToggle) {
                airplaneLauncher.position = 1.0
            } else {
                airplaneLauncher.position = 0.0
            }
        }
    }

    protected fun handleWinch() {
        latestWinchPosition = winch.currentPosition
        winch.power = gamepad.right_stick_y.pow(2).toDouble() * WINCH_SPEED
        gamepadManager.once("left_stick_y") { allowWinchMovement = !allowWinchMovement }
        if (allowWinchMovement) {
            winch.mode = DcMotor.RunMode.RUN_USING_ENCODER
            winch.power = gamepad.right_stick_y.pow(2).toDouble() * WINCH_SPEED
        } else {
            winch.mode = DcMotor.RunMode.RUN_TO_POSITION
            winch.targetPosition = latestWinchPosition
            winch.power = 0.2
        }
    }


    protected fun handleAllRobotBits() {
        handleOvercurrentProtection()
        handlePixelDroppers()
        handleAirplaneLauncher()
        handleWinch()
    }

    protected companion object {
        // Constants
        const val ARM_ENCODER_RES = 2786.2 // PPR
        const val ARM_SPEED = 0.4
        const val ARM_OVERCURRENT_THRESHOLD = 4.0 // Amps
        const val WINCH_SPEED = 0.5
    }
}