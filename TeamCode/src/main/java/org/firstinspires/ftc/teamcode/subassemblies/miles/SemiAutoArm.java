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
public class SemiAutoArm {

    // References to gamepad, telemetry, and hardware map
    public Gamepad _gamepad;
    public Telemetry _telemetry;
    public HardwareMap _hardwareMap;

    // Constants for arm control
    private static final double ARM_ENCODER_RESOLUTION = 2786.2; // in pulses per rotation
    private static final double ARM_POWER = 0.4;
    private static final double ARM_OVERCURRENT_THRESHOLD = 4;

    // Position presets
    private static final int ARM_HORIZONTAL_POSITION = 1550;
    private static final int ARM_WINCH_POSITION = 0; // TODO: get value for this
    private static final double WRIST_WINCH_POSITION = 0; // TODO: get value for this

    // Math constants
    private static final double GAMMA = Math.atan(16.0 / 283.0) * (180 / Math.PI);

    public ElapsedTime loopTime = new ElapsedTime();

    // Motor and servo instances for the semi-auto arm
    public DcMotorEx arm;
    public Servo leftRelease;
    public Servo rightRelease;
    public Servo wrist;

    // Variables for tracking arm position, wrist angle, and release statuses
    private int latestArmPosition = ARM_HORIZONTAL_POSITION;
    private double theta;
    private String leftReleaseStatus;
    private String rightReleaseStatus;
    private String wristAlignment;
    private boolean isDriverArmMovementLocked = false;
    public boolean needsStop = false;

    // Initializes the semi-auto arm subassembly
    public void init() {

        arm = _hardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = _hardwareMap.get(Servo.class, "left_release");
        rightRelease = _hardwareMap.get(Servo.class, "right_release");
        wrist = _hardwareMap.get(Servo.class, "wrist");

        // Configuring arm motor
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS); // Threshold for current usage alert

        // Configuring servos with appropriate ranges and directions
        leftRelease.scaleRange(0.15, 0.40); // 22.5% of 300-degree range
        leftRelease.setDirection(Servo.Direction.FORWARD);

        rightRelease.scaleRange(0.6, 1); // 22.5% of 300-degree range
        rightRelease.setDirection(Servo.Direction.REVERSE);

        wrist.scaleRange(0.25, 0.78); // 53% of 300-degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        // Initializing variables
        leftReleaseStatus = "unknown";
        rightReleaseStatus = "unknown";
        wristAlignment = "easel";
        theta = 60;

        _telemetry.addData(">", "Semi-Auto Arm Ready.");
    }

    // Main loop for controlling the semi-auto arm
    public void loop() {

        loopTime.reset();

        ifOvercurrentProtectArm();

        // Arm movement
        if (!isDriverArmMovementLocked) {
            if (_gamepad.left_stick_y != 0) {
                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                arm.setPower(_gamepad.left_stick_y * ARM_POWER);
                latestArmPosition = arm.getCurrentPosition();
            } else {
                brakeArm();
            }
        }

        // Wrist movement
        wrist.setPosition(findWristPosition());

        // Intake
        if (_gamepad.left_bumper) {
            leftRelease.setPosition(1);
            leftReleaseStatus = "open";
        } else if (_gamepad.left_trigger > 0.2) {
            leftRelease.setPosition(0);
            leftReleaseStatus = "closed";
        }
        if (_gamepad.right_bumper) {
            rightRelease.setPosition(1);
            rightReleaseStatus = "open";
        } else if (_gamepad.right_trigger > 0.2) {
            rightRelease.setPosition(0);
            rightReleaseStatus = "closed";
        }

        // Reset arm position
        if (_gamepad.b) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // Wrist alignment
        if (_gamepad.a) {
            theta = 120;
            wristAlignment = "floor";
        } else if (_gamepad.y) {
            theta = 60;
            wristAlignment = "easel";
        }
        // Hook onto bar for winching
        if (_gamepad.x) {
            isDriverArmMovementLocked = true;
            wrist.setPosition(WRIST_WINCH_POSITION);
            arm.setTargetPosition(ARM_WINCH_POSITION);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (_gamepad.left_stick_button) {
            isDriverArmMovementLocked = false;
        }
    }

    // Displays relevant telemetry information
    public void telemetry() {

        // Telemetry updates
        _telemetry.addData("Semi-Automatic Arm", "");
        _telemetry.addData("loop time (milliseconds)", loopTime.milliseconds());
        _telemetry.addData("is arm busy", arm.isBusy());
        _telemetry.addData("arm mode", arm.getMode());
        _telemetry.addData("arm position", arm.getCurrentPosition());
        _telemetry.addData("arm target position", arm.getTargetPosition());
        _telemetry.addData("arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition());
        _telemetry.addData("wrist position", wrist.getPosition());
        _telemetry.addData("wrist alignment", wristAlignment);
        _telemetry.addData("left release status", leftReleaseStatus);
        _telemetry.addData("right release status", rightReleaseStatus);
        _telemetry.addData("arm motor current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        _telemetry.addData("is arm over current", arm.isOverCurrent());
        _telemetry.addData("is arm driver arm movement locked", isDriverArmMovementLocked);
        _telemetry.addLine();
    }

    // Calculates the wrist position based on arm angle and theta
    private double findWristPosition() {
        double armAngle = 360 * arm.getCurrentPosition() / ARM_ENCODER_RESOLUTION;
        double servoAngle = 90 + theta - GAMMA - armAngle;
        return (servoAngle - 90) / (0.53 * 300) - (0.5 * 0.53);
    }

    // Brakes the arm motor to maintain position
    private void brakeArm() {
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    // Checks and handles overcurrent conditions for the arm motor
    private void ifOvercurrentProtectArm() {
        if (arm.isOverCurrent()) {
            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                needsStop = true; // Request stop of the OpMode, controlled in SemiAutoTeleOp.java
            } else {
                isDriverArmMovementLocked = true;
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else {
            isDriverArmMovementLocked = false;
        }
    }
}