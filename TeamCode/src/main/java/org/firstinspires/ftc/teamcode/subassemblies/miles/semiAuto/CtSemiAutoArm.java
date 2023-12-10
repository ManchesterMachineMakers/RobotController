package org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto;

import java.lang.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.manchestermachinemakers.easyop.Device;
import org.manchestermachinemakers.easyop.Subassembly;

// Comments courtesy of ChatGPT
public class CtSemiAutoArm implements Subassembly {

    public OpMode opMode;

    // constructor method
    public CtSemiAutoArm(OpMode opMode) {
        this.opMode = opMode;
    }

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
    @Device("arm") public DcMotorEx arm;
    @Device("wrist") public Servo wrist;
    @Device("left_release") public Servo leftRelease;
    @Device("right_release") public Servo rightRelease;

    // Variables for tracking arm position, wrist angle, and release statuses
    private int latestArmPosition = ARM_HORIZONTAL_POSITION;
    private double theta = 60;
    private String wristAlignment = "easel";

    public String currentStatus = "unknown";
    public double runTime = 0;

    // Initializes the semi-auto arm subassembly
    public void init() {

        gamepad = opMode.gamepad2;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        PIDFCoefficients armPIDF = new PIDFCoefficients(ARM_P, ARM_I, ARM_D, ARM_F);

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

        telemetry.addData(">", "Semi-Auto Arm Ready.");
    }

    // Main loop for controlling the semi-auto arm
    public void loop() {

        loopTime.reset();

        // Arm movement
        if (!arm.isOverCurrent()) {
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
        } else if (gamepad.left_trigger > 0.2) {
            leftRelease.setPosition(0);
        }
        if (gamepad.right_bumper) {
            rightRelease.setPosition(1);
        } else if (gamepad.right_trigger > 0.2) {
            rightRelease.setPosition(0);
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
            wrist.setPosition(WRIST_WINCH_POSITION);
            arm.setTargetPosition(ARM_WINCH_POSITION);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        telemetry.addData("arm velocity", arm.getVelocity());
        telemetry.addData("wrist alignment", wristAlignment);
        telemetry.addData("arm motor current (amps)", arm.getCurrent(CurrentUnit.AMPS));
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
    public void overcurrentProtection() {

        if (arm.isOverCurrent()) {

            telemetry.addData("WARNING", "arm motor is overcurrent, reduce load or the arm may break");

            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                opMode.requestOpModeStop();
            } else {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}