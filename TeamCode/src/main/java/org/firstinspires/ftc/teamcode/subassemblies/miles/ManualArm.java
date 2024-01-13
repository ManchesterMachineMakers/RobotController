package org.firstinspires.ftc.teamcode.subassemblies.miles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Comments courtesy of ChatGPT
public class ManualArm {

    // Constants for arm control
    private static final double ARM_SPEED = 0.5;
    private static final double ARM_OVERCURRENT_THRESHOLD = 4;

    // References to gamepad, telemetry, and hardware map
    public Gamepad gamepad;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    private final ElapsedTime loopTime = new ElapsedTime();

    // Motor and servo instances for the manual arm
    public DcMotorEx arm;
    public Servo leftRelease;
    public Servo rightRelease;
    public Servo wrist;

    // Variables for tracking arm and wrist positions, release statuses, and button states
    private String leftReleaseStatus;
    private String rightReleaseStatus;
    private int latestArmPosition;
    private double wristPosition;
    private boolean buttonWasPressed;
    public boolean needsStop;

    // Initializes the manual arm subassembly
    public void init() {

        wristPosition = wrist.getPosition();

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = hardwareMap.get(Servo.class, "left_release");
        rightRelease = hardwareMap.get(Servo.class, "right_release");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Configuring arm motor
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS);

        // Configuring servos with appropriate ranges and directions
        leftRelease.scaleRange(0.175, 0.4); // 22.5% of 300 degree range
        leftRelease.setDirection(Servo.Direction.FORWARD);

        rightRelease.scaleRange(0.6, 0.825); // 22.5% of 300 degree range
        rightRelease.setDirection(Servo.Direction.REVERSE);

        wrist.scaleRange(0.25, 0.78); // // 53% of 300 degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        // Initializing variables
        leftReleaseStatus = "unknown";
        rightReleaseStatus = "unknown";
        latestArmPosition = arm.getCurrentPosition();
        buttonWasPressed = false;

        telemetry.addData(">", "Arm Subassembly Ready.");
    }

    // Main loop for controlling the manual arm
    public void loop() {
        loopTime.reset(); // Keep track of time spent in each loop for debugging

        protectArmIfOverCurrent();

        // Wrist control
        wristPosition = wrist.getPosition();
        wristPosition += gamepad.right_stick_y / 25;
        if (wristPosition < 0) {
            wristPosition = 0;
        } else if (wristPosition > 1) {
            wristPosition = 1;
        }
        if (gamepad.left_stick_y != 0.0) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(-gamepad.left_stick_y * ARM_SPEED);
            latestArmPosition = arm.getCurrentPosition();
        } else {
            // brake
            arm.setTargetPosition(latestArmPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Wrist control with buttons
        if (!buttonWasPressed) {
            if (gamepad.dpad_up) {
                wristPosition -= 0.05;
            } else if (gamepad.dpad_down) {
                wristPosition += 0.05;
            } else if (gamepad.dpad_left) {
                wristPosition += 0.2;
            } else if (gamepad.dpad_right) {
                wristPosition -= 0.2;
            }
        }
        wrist.setPosition(wristPosition);

        if (gamepad.b) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Pixel release mechanism (brush)
        // Left
        if (gamepad.left_bumper) { // Open
            leftRelease.setPosition(1);
            leftReleaseStatus = "open";
        } else if (gamepad.left_trigger > 0.2) { // Close
            leftRelease.setPosition(0);
            leftReleaseStatus = "closed";
        }
        // Right
        if (gamepad.right_bumper) { // Open
            rightRelease.setPosition(1);
            rightReleaseStatus = "open";
        } else if (gamepad.right_trigger > 0.2) { // Close
            rightRelease.setPosition(0);
            rightReleaseStatus = "closed";
        }

        // For detecting when a button is pressed.
        buttonWasPressed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right;
    }

    // Displays relevant telemetry information
    public void telemetry() {
        telemetry.addData("Manual Arm", "");
        telemetry.addData("loop time (nanoseconds)", loopTime.nanoseconds());
        telemetry.addData("arm mode", arm.getMode());
        telemetry.addData("arm target position", arm.getTargetPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition());
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("left release position", leftReleaseStatus);
        telemetry.addData("right release position", rightReleaseStatus);
        telemetry.addData("arm current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
    }


    // Checks and handles overcurrent conditions for the arm motor
    private void protectArmIfOverCurrent() {
        if (arm.isOverCurrent()) {
            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                needsStop = true; // request stop of the opMode
            } else {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}