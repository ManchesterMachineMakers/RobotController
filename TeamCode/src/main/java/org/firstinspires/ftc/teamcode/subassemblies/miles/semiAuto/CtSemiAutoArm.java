package org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto;

import java.lang.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Comments courtesy of ChatGPT
public class CtSemiAutoArm {

    // References to gamepad, telemetry, and hardware map
    public Gamepad gamepad;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // Constants for arm control
    private static final double ARM_ENCODER_RESOLUTION = 2786.2; // in pulses per rotation
    private static final double ARM_SPEED = 0.4;
    private static final double ARM_OVERCURRENT_THRESHOLD = 4;

    // PIDF Coefficients
    private static final double ARM_P = 10; // proportion
    private static final double ARM_I = 0; // integral
    private static final double ARM_D = 0; // derivative
    private static final double ARM_F = 2;

    // Position presets
    private static final int ARM_HORIZONTAL_POSITION = 1550;
    private static final int ARM_WINCH_POSITION = 0; // TODO: get value for this
    private static final double WRIST_WINCH_POSITION = 0; // TODO: get value for this

    // Math constants
    private static final double GAMMA = Math.atan(16.0 / 283.0) * (180 / Math.PI);

    public ElapsedTime loopTime = new ElapsedTime();

    // Motor and servo instances for the semi-auto arm
    public DcMotorEx arm;
    private Servo leftRelease;
    private Servo rightRelease;
    private Servo wrist;

    // Variables for tracking arm position, wrist angle, and release statuses
    private int latestArmPosition = ARM_HORIZONTAL_POSITION;
    private double theta;
    private String leftReleaseStatus;
    private String rightReleaseStatus;
    private String wristAlignment;
    private boolean isDriverArmMovementLocked = false;

    public boolean needsStop = false;
    public String currentStatus = "unknown";
    public double runTime = 0;

    // Initializes the semi-auto arm subassembly
    public void init() {
        PIDFCoefficients armPIDF = new PIDFCoefficients(ARM_P, ARM_I, ARM_D, ARM_F);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = hardwareMap.get(Servo.class, "left_release");
        rightRelease = hardwareMap.get(Servo.class, "right_release");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Configuring arm motor
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS); // Threshold for current usage alert
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, armPIDF);

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
        theta = 60; // 60 for easel, 120 for floor

        telemetry.addData(">", "Semi-Auto Arm Ready.");
    }

    // Main loop for controlling the semi-auto arm
    public void loop() {

        loopTime.reset();

        ifOvercurrentProtectArm();

        // Arm movement
        if (!isDriverArmMovementLocked) {
            if (gamepad.left_stick_y != 0) {
                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                arm.setVelocity(gamepad.left_stick_y * ARM_ENCODER_RESOLUTION * ARM_SPEED);
                latestArmPosition = arm.getCurrentPosition();
            } else {
                brakeArm();
            }
        }

        // Wrist movement
        wrist.setPosition(findWristPosition());

        // Intake servo movement
        if (gamepad.left_bumper) {
            leftRelease.setPosition(1);
            leftReleaseStatus = "open";
        } else if (gamepad.left_trigger > 0.2) {
            leftRelease.setPosition(0);
            leftReleaseStatus = "closed";
        }
        if (gamepad.right_bumper) {
            rightRelease.setPosition(1);
            rightReleaseStatus = "open";
        } else if (gamepad.right_trigger > 0.2) {
            rightRelease.setPosition(0);
            rightReleaseStatus = "closed";
        }

        // Reset arm position
        if (gamepad.b) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // Wrist alignment
        if (gamepad.a) {
            theta = 120;
            wristAlignment = "floor";
        } else if (gamepad.y) {
            theta = 60;
            wristAlignment = "easel";
        }

        // Hook onto bar for winching
        if (gamepad.x) {
            isDriverArmMovementLocked = true;
            wrist.setPosition(WRIST_WINCH_POSITION);
            arm.setTargetPosition(ARM_WINCH_POSITION);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Manually unlock driver movement
        if (gamepad.left_stick_button) {
            isDriverArmMovementLocked = false;
        }
    }

    // Displays relevant telemetry information
    public void telemetry() {

        // Telemetry updates
        telemetry.addData("Semi-Automatic Arm", "");
        telemetry.addData("status", currentStatus);
        telemetry.addData("run time", (int) runTime);
        telemetry.addData("loop time (milliseconds)", (int) loopTime.milliseconds());
        telemetry.addData("arm mode", arm.getMode());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("arm target position", arm.getTargetPosition());
        telemetry.addData("arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition());
        telemetry.addData("wrist alignment", wristAlignment);
        telemetry.addData("$leftR", leftReleaseStatus);
        telemetry.addData("right release status", rightReleaseStatus);
        telemetry.addData("arm motor current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("is arm driver arm movement locked", isDriverArmMovementLocked);
        telemetry.addLine();
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
            telemetry.addData("\033[31WARNING", "arm motor is overcurrent, reduce load or the arm may break");

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