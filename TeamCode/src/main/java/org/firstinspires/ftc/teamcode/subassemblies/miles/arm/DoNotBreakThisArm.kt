package org.firstinspires.ftc.teamcode.subassemblies.miles.arm

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

// Comments courtesy of ChatGPT
class DoNotBreakThisArm(private val opMode: OpMode, private val gamepad: Gamepad = opMode.gamepad2) {

    private val name = "Manual Arm"
    private var status = "uninitialized"
    private val telemetry = opMode.telemetry
    private val hardwareMap = opMode.hardwareMap
    private val loopTime = ElapsedTime()

    // Motors
    private val arm = hardwareMap.get(DcMotorEx::class.java, "arm")
//    private val winch = hardwareMap.dcMotor.get("winch")

    // Servos
    private val leftRelease = hardwareMap.servo.get("left_release")
    private val rightRelease = hardwareMap.servo.get("right_release")
    private val wrist = hardwareMap.servo.get("wrist")

    // Variables for tracking arm and wrist positions, release statuses, and button states
    private var latestArmPosition = 0
    private var wristPosition = 0.0
    private var buttonWasPressed = false

    init {
        wristPosition = wrist.position

        // Configuring arm motor
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.direction = DcMotorSimple.Direction.FORWARD
        arm.mode = DcMotor.RunMode.RUN_USING_ENCODER
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS)

        // Configuring servos with appropriate ranges and directions
        leftRelease.scaleRange(0.175, 0.4) // 22.5% of 300 degree range
        leftRelease.direction = Servo.Direction.FORWARD
        rightRelease.scaleRange(0.6, 0.825) // 22.5% of 300 degree range
        rightRelease.direction = Servo.Direction.REVERSE
        wrist.scaleRange(0.25, 0.78) // // 53% of 300 degree range
        wrist.direction = Servo.Direction.FORWARD

        // Initializing variables
        latestArmPosition = arm.currentPosition
        buttonWasPressed = false
        telemetry.addData(">", "Arm Subassembly Ready.")
    }

    // Main loop for controlling the manual arm
    fun loop() {
        loopTime.reset() // Keep track of time spent in each loop for debugging
        handleOvercurrentProtection()

        // Wrist control
        wristPosition = wrist.position
        wristPosition += (gamepad.right_stick_y / 25).toDouble()
        if (wristPosition < 0) {
            wristPosition = 0.0
        } else if (wristPosition > 1) {
            wristPosition = 1.0
        }
        if (gamepad.left_stick_y.toDouble() != 0.0) {
            arm.mode = DcMotor.RunMode.RUN_USING_ENCODER
            arm.power = -gamepad.left_stick_y * ARM_SPEED
            latestArmPosition = arm.currentPosition
        } else {
            // brake
            arm.targetPosition = latestArmPosition
            arm.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        // Wrist control with buttons
        if (!buttonWasPressed) {

            if      (gamepad.dpad_up)    wristPosition -= 0.05
            else if (gamepad.dpad_down)  wristPosition += 0.05
            else if (gamepad.dpad_left)  wristPosition += 0.2
            else if (gamepad.dpad_right) wristPosition -= 0.2
        }
        wrist.position = wristPosition

        if (gamepad.b) arm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Pixel release mechanism (brush)
        // Left
        if (gamepad.left_bumper) leftRelease.position = 1.0 // open
        else if (gamepad.left_trigger > 0.2) leftRelease.position = 0.0 // close
        // Right
        if (gamepad.right_bumper) rightRelease.position = 1.0 // open
        else if (gamepad.right_trigger > 0.2) rightRelease.position = 0.0 // close


        // For detecting when a button is pressed.
        buttonWasPressed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right
    }

    // Displays relevant telemetry information
    fun telemetry() {
        telemetry.addData(name, status)
        telemetry.addData("loop time (nanoseconds)", loopTime.nanoseconds())
        telemetry.addData("arm mode", arm.mode)
        telemetry.addData("arm target position", arm.targetPosition)
        telemetry.addData("arm position", arm.currentPosition)
        telemetry.addData("arm position discrepancy", arm.currentPosition - arm.targetPosition)
        telemetry.addData("wrist position", wrist.position)
        telemetry.addData("arm current (amps)", arm.getCurrent(CurrentUnit.AMPS))
        telemetry.addLine()
    }

    // Checks and handles overcurrent conditions for the arm motor
    private fun handleOvercurrentProtection() {
        if (arm.isOverCurrent) {
            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                opMode.requestOpModeStop()
            } else {
                arm.targetPosition = 0
                arm.mode = DcMotor.RunMode.RUN_TO_POSITION
            }
        }
    }

    companion object {
        // Constants for arm control
        private const val ARM_SPEED = 0.5
        private const val ARM_OVERCURRENT_THRESHOLD = 4.0
    }
}