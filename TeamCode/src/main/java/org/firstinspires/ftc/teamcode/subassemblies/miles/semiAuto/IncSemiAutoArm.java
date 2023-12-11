package org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto;

import java.lang.*;
import java.util.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.BasicArm;

// Comments courtesy of ChatGPT
public class IncSemiAutoArm extends BasicArm {

    private static final int ARM_LARGE_INCREMENT = 4;
    private static final int ARM_INCREMENT_UPPER_LIMIT = 7;

    // Position presets
    private static final int ARM_WINCH_POSITION = 0; // TODO: get value for this
    private static final double WRIST_WINCH_POSITION = 0; // TODO: get value for this

    // Math constants
    private static final double GAMMA = Math.atan(16.0 / 283.0) * (180 / Math.PI);
    private static final double D1 = 220;
    private static final double D2 = 88.9;
    private static final double L2 = 67.88;
    private static final double L3 = 59.5;
    private static final double H = 287.75;
    private static final double R = 336;

    private double theta = 60; // 60 for easel, 120 for floor
    private String wristAlignment;
    private int pixelLayer = 0;

    public IncSemiAutoArm(OpMode opMode) { super(opMode); } // constructor

    // Main loop for controlling the semi-auto arm
    @Override public void loop() {

        loopTime.reset();

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
        if (gamepad.dpad_up && !buttonWasPressed) {
            pixelLayer++;
        } else if (gamepad.dpad_down && !buttonWasPressed) {
            pixelLayer = -1;
        } else if (gamepad.dpad_right && !buttonWasPressed) {
            pixelLayer += ARM_LARGE_INCREMENT;
        } else if (gamepad.dpad_left && !buttonWasPressed) {
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

        buttonWasPressed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right;
    }

    // Displays relevant telemetry information
    @Override public void telemetry() {

        // Telemetry updates
        telemetry.addData("Semi-Automatic Arm", "");
        telemetry.addData("status", currentStatus);
        telemetry.addData("run time", (int) opMode.getRuntime());
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
        int targetArmPosition = (int) (alpha / 360 * ARM_ENCODER_RES); // from degrees to encoder ticks
        double targetWristPosition = beta * 0.53 * 300 / 360; // from degrees to the servo's range (53% of 300 degrees)
        return new AbstractMap.SimpleEntry<>(targetArmPosition, targetWristPosition);
    }

}