package org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto;

import java.lang.*;
import java.util.*;

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
public class IncSemiAutoArm {

    // References to gamepad, telemetry, and hardware map
    public Gamepad gamepad;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // Constants for arm control
    private static final double ARM_ENCODER_RESOLUTION = 2786.2; // in pulses per rotation
    private static final double ARM_SPEED = 0.4;
    private static final double ARM_OVERCURRENT_THRESHOLD = 4;
    private static final int ARM_LARGE_INCREMENT = 4;
    private static final int ARM_INCREMENT_UPPER_LIMIT = 7;

    // Position presets
    private static final int ARM_WINCH_POSITION = 0; // TODO: get value for this
    private static final double WRIST_WINCH_POSITION = 0; // TODO: get value for this

    // Math constants for getWristAndArmPosition()
    private static final double
            GAMMA = Math.atan(16.0 / 283.0) * (180 / Math.PI),
            D1 = 220,
            D2 = 88.9,
            L2 = 67.88,
            L3 = 59.5,
            H = 287.75,
            R = 336;

    public ElapsedTime loopTime = new ElapsedTime();

    // Motor and servo instances for the semi-auto arm
    public DcMotorEx arm;
    private Servo leftRelease;
    private Servo rightRelease;
    private Servo wrist;

    // Variables for tracking arm position, wrist angle, and release statuses
    private double theta = 60; // 60 for easel, 120 for floor
    private String wristAlignment;
    private int pixelLayer = 0;
    private boolean dpadWasUsed = false;

    public boolean needsStop = false;
    public String currentStatus = "unknown";
    public double runTime = 0;

    // Initializes the semi-auto arm subassembly
    public void init() {

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = hardwareMap.get(Servo.class, "left_release");
        rightRelease = hardwareMap.get(Servo.class, "right_release");
        wrist = hardwareMap.get(Servo.class, "wrist");

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
        wristAlignment = "easel";


        telemetry.addData("getArmAndWristPosition().first", getTargetArmAndWristPositions().getKey());
        telemetry.addData("getArmAndWristPosition().second", getTargetArmAndWristPositions().getValue());

        telemetry.addData(">", "Semi-Auto Arm Ready.");
    }

    // Main loop for controlling the semi-auto arm
    public void loop() {

        loopTime.reset();

        ifOvercurrentProtectArm();

        arm.setTargetPosition(getTargetArmAndWristPositions().getKey());
        wrist.setPosition(getTargetArmAndWristPositions().getValue());

        // Intake servo movement
        if (gamepad.left_bumper) {
            leftRelease.setPosition(1);
        } else if (gamepad.left_trigger > 0.2) {
            leftRelease.setPosition(0);
        }
        if (gamepad.right_bumper) {
            rightRelease.setPosition(1);
        } else if (gamepad.right_trigger > 0.2) {
            rightRelease.setPosition(0);
        }

        // Incrementer
        if (gamepad.dpad_up && !dpadWasUsed) {
            pixelLayer++;
        } else if (gamepad.dpad_down && !dpadWasUsed) {
            pixelLayer = -1;
        } else if (gamepad.dpad_right && !dpadWasUsed) {
            pixelLayer += ARM_LARGE_INCREMENT;
        } else if (gamepad.dpad_left && !dpadWasUsed) {
            pixelLayer -= ARM_LARGE_INCREMENT;
        }
        // pixelLayer must stay between -1 and 7
        if (pixelLayer < -1) {
            pixelLayer = -1;
        } else if (pixelLayer > ARM_INCREMENT_UPPER_LIMIT) {
            pixelLayer = ARM_INCREMENT_UPPER_LIMIT;
        }

        // Reset arm position
        if (gamepad.b) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            wrist.setPosition(WRIST_WINCH_POSITION);
            arm.setTargetPosition(ARM_WINCH_POSITION);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        dpadWasUsed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right;
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
        telemetry.addData("arm pixel layer", pixelLayer);
        telemetry.addData("wrist alignment", wristAlignment);
        telemetry.addData("arm motor current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
    }

    // See math: https://drive.google.com/file/d/1ADeKl-3EPOc8nBHZwGThREwBQAEdIAJ9/view
    // pixelStack == n
    private Map.Entry<Integer, Double> getTargetArmAndWristPositions() {

        double argument = ((D1 + pixelLayer * D2 - L3) * Math.sin(theta) + L2 * Math.cos(theta) - H) / R;
        double alpha = Math.asin(argument);
        telemetry.addData("argument", argument);
        telemetry.addData("alpha", alpha);
        if (alpha < -1) {
            alpha = -1;
        } else if (alpha > 1) {
            alpha = 1;
        }
        double beta;
        if (pixelLayer <= -1) { // floor
            beta = 0.5;
        } else { // easel
            beta = theta - GAMMA - alpha;
        }
        int targetArmPosition = (int) (alpha / 360 * ARM_ENCODER_RESOLUTION); // from degrees to encoder ticks
        double targetWristPosition = beta * 0.53 * 300 / 360; // from degrees to the servo's range (53% of 300 degrees)
        return new AbstractMap.SimpleEntry<>(targetArmPosition, targetWristPosition);
    }

    // Checks and handles overcurrent conditions for the arm motor
    private void ifOvercurrentProtectArm() {

        if (arm.isOverCurrent()) {
            telemetry.addData("WARNING", "arm motor is overcurrent, reduce load or the arm may break");

            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                needsStop = true; // Request stop of the OpMode, controlled in SemiAutoTeleOp.java
            } else {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}