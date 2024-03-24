package org.firstinspires.ftc.teamcode.subassemblies.misc

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

class DoNotBreakThisArm(private val opMode: OpMode, private val gamepad: Gamepad = opMode.gamepad2) {

    private val name = "Manual Arm"
    private val telemetry = opMode.telemetry
    private val hardwareMap = opMode.hardwareMap
    private val loopTime = ElapsedTime()

    // Motors
    private val arm = hardwareMap.dcMotor.get("arm")
    private val winch = hardwareMap.dcMotor.get("winch")

    // Servos
    private val leftRelease = hardwareMap.servo.get("left_release")
    private val rightRelease = hardwareMap.servo.get("right_release")
    private val wrist = hardwareMap.servo.get("wrist")
    private val airplaneLauncher = hardwareMap.servo.get("airplane_launcher")

    // Variables for tracking arm and wrist positions, release statuses, and button states
    private var wristPosition = 0.0
    private var buttonWasPressed = false

    init {
        wristPosition = wrist.position

        // Configuring arm motor
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        arm.direction = DcMotorSimple.Direction.FORWARD
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // Configuring servos with appropriate ranges and directions
        leftRelease.scaleRange(0.15, 0.40) // 22.5% of 300 degree range
        leftRelease.direction = Servo.Direction.FORWARD

        rightRelease.scaleRange(0.4, 0.85) // 22.5% of 300 degree range
        rightRelease.direction = Servo.Direction.REVERSE

        wrist.scaleRange(0.25, 0.78) // // 53% of 300 degree range
        wrist.direction = Servo.Direction.FORWARD

        airplaneLauncher.scaleRange(0.7, 0.78) // 1 should be open, 0 should be closed
        airplaneLauncher.direction = Servo.Direction.REVERSE

        // Initializing variables
        buttonWasPressed = false
        telemetry.addData(">", "Arm Subassembly Ready.")
    }

    // Main loop for controlling the manual arm
    fun loop() {
        loopTime.reset() // Keep track of time spent in each loop for debugging

        // Wrist control
        wristPosition = wrist.position
        if (wristPosition < 0) {
            wristPosition = 0.0
        } else if (wristPosition > 1) {
            wristPosition = 1.0
        }

        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        arm.power = -gamepad.left_stick_y * ARM_SPEED

        winch.power = gamepad.right_stick_y.toDouble()

        // Wrist control with dpad
        if (!buttonWasPressed) {
            wristPosition += when {
                gamepad.dpad_up -> -0.05
                gamepad.dpad_down -> 0.05
                gamepad.a -> 0.2
                gamepad.y -> -0.2
                else -> 0.0
            }
        }

        wrist.position = wristPosition

        // Pixel release mechanism (brush)
        // Left
        if (gamepad.left_bumper) leftRelease.position = 1.0 // open
        else if (gamepad.left_trigger > 0.2) leftRelease.position = 0.0 // close
        // Right
        if (gamepad.right_bumper) rightRelease.position = 1.0 // open
        else if (gamepad.right_trigger > 0.2) rightRelease.position = 0.0 // close

        // Airplane Launcher
        if(gamepad.x) airplaneLauncher.position = 1.0
        if(gamepad.b) airplaneLauncher.position = 0.0

        // For detecting when a button is pressed.
        buttonWasPressed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right
    }

    // Displays relevant telemetry information
    fun telemetry() {
        telemetry.addLine(name)
        telemetry.addData("loop time (nanoseconds)", loopTime.nanoseconds())
        telemetry.addData("arm mode", arm.mode)
        telemetry.addData("arm target position", arm.targetPosition)
        telemetry.addData("arm position", arm.currentPosition)
        telemetry.addData("arm position discrepancy", arm.currentPosition - arm.targetPosition)
        telemetry.addData("wrist position", wrist.position)
        telemetry.addLine()
    }

    companion object {
        private const val ARM_SPEED = 1.0
    }
}