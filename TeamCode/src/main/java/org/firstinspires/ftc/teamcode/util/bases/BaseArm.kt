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
abstract class BaseArm(opMode: OpMode, gamepad: Gamepad, name: String) : Subassembly(opMode, gamepad, name) {

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
    private var airplaneLauncherToggle = false // false = closed, true = open
    // Button register
    private var registerX = false

    init {
        status = "initializing"

        // Arm motor configuration
        arm.direction = DcMotorSimple.Direction.REVERSE
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS)

        // Winch motor configuration
        winch.direction = DcMotorSimple.Direction.FORWARD
        winch.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Wrist servo configuration
        wrist.scaleRange(0.25, 0.78) // 53% of 300-degree range
        wrist.direction = Servo.Direction.FORWARD

        // Arm servo configuration
        airplaneLauncher.scaleRange(0.7, 0.78) // 0: closed, 1: open
        airplaneLauncher.direction = Servo.Direction.REVERSE

        // Left release servo configuration
        leftRelease.scaleRange(0.15, 0.40) // 22.5% of 300-degree range
        leftRelease.direction = Servo.Direction.FORWARD

        // Right release servo configuration
        rightRelease.scaleRange(0.6, 1.0) // 22.5% of 300-degree range
        rightRelease.direction = Servo.Direction.REVERSE

        // Display initialization completion message
        telemetry.addData(">", "Arm Subassembly Ready")
    }


    // Runs once after start is pressed, before loop()
    open fun start() {
        airplaneLauncher.position = 0.0 // ensure launcher is closed
    }

    abstract override fun loop()

    override fun telemetry() {
        super.telemetry()
        telemetry.addData("arm mode", arm.mode)
        telemetry.addData("arm current (amps)", arm.getCurrent(CurrentUnit.AMPS).toString() + " out of : " + ARM_OVERCURRENT_THRESHOLD)
        telemetry.addData("arm position", arm.currentPosition)
    }

    /**
     * Check for overcurrent condition and take appropriate action.
     */
    private fun handleOvercurrentProtection() {
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
    private fun handlePixelDroppers() {
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

    private fun handleAirplaneLauncher() {
        // Airplane launcher
        if(gamepad.x && !registerX) {
            registerX = true
            airplaneLauncherToggle = !airplaneLauncherToggle

            if (airplaneLauncherToggle) airplaneLauncher.position = 1.0 // Open
            else airplaneLauncher.position = 0.0 // Close
        }
        else if (!gamepad.x) registerX = false
    }

    private fun handleWinch() {
        val rightY = gamepad.right_stick_y
        // Power curve to increase winch sensitivity, without reducing speed
        winch.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        winch.power =
            if (rightY > 0.0) rightY.pow(2) * WINCH_POWER
            else -rightY.pow(2) * WINCH_POWER
    }

    protected fun handleAllRobotBits() {
        handleOvercurrentProtection()
        handlePixelDroppers()
        handleAirplaneLauncher()
        handleWinch()
    }


    protected companion object {
        // Constants
        const val ARM_ENCODER_RES = 2786.2 * 2 // PPR of motor * 2:1 gearing ratio
        const val ARM_POWER = 0.8
        const val ARM_OVERCURRENT_THRESHOLD = 5.0 // Amps
        const val WINCH_POWER = 1.0
    }
}