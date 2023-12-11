package org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto;

import java.lang.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.BasicArm;

// Comments courtesy of ChatGPT
public class CtSemiAutoArm extends BasicArm {

    // Position presets
    private static final int ARM_HORIZONTAL_POSITION = 1550; // encoder ticks (PPR)
    private static final int ARM_WINCH_POSITION = 0; // TODO: get value for this
    private static final double WRIST_WINCH_POSITION = 0; // TODO: get value for this

    private static final double GAMMA = Math.atan(16.0 / 283.0) * (180 / Math.PI); // for math

    private double theta = 60; // degrees
    private String wristAlignment = "easel";

    public CtSemiAutoArm(OpMode opMode) { super(opMode); }

    // Main loop for controlling the semi-auto arm
    @Override public void loop() {

        loopTime.reset();

        // Arm movement
        if (!arm.isOverCurrent()) {
            if (gamepad.left_stick_y != 0) {
                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                arm.setVelocity(gamepad.left_stick_y * ARM_ENCODER_RES * ARM_SPEED);
                latestArmPosition = arm.getCurrentPosition();
            } else {
                brakeArm();
            }
        }

        // Wrist movement
        wrist.setPosition(findWristPosition());

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
            wrist.setPosition(WRIST_WINCH_POSITION);
            arm.setTargetPosition(ARM_WINCH_POSITION);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        controlIntakeServos();
    }

    // Displays relevant telemetry information
    @Override public void telemetry() {

        // Telemetry updates
        telemetry.addData("Semi-Automatic Arm", "");
        telemetry.addData("status", currentStatus);
        telemetry.addData("run time", (int) opMode.getRuntime());
        telemetry.addData("loop time (milliseconds)", (int) loopTime.milliseconds());
        telemetry.addData("arm mode", arm.getMode());
        telemetry.addData("arm velocity", arm.getVelocity());
        telemetry.addData("wrist alignment", wristAlignment);
        telemetry.addData("arm motor current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
    }

    // Calculates the wrist position based on arm angle and theta
    private double findWristPosition() {
        double armAngle = 360 * arm.getCurrentPosition() / ARM_ENCODER_RES;
        double servoAngle = 90 + theta - GAMMA - armAngle;
        return (servoAngle - 90) / (0.53 * 300) - (0.5 * 0.53);
    }
}