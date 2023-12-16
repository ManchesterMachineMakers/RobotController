package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.manchestermachinemakers.easyop.Device;
import org.manchestermachinemakers.easyop.Subassembly;

/**
 * Abstract class representing a basic arm subassembly for FTC robotics.
 */
public abstract class BaseArm implements Subassembly {

    // Constants
    protected static final double ARM_ENCODER_RES = 2786.2; // PPR
    protected static final double ARM_SPEED = 0.4;
    protected static final double ARM_OVERCURRENT_THRESHOLD = 4; // Amps

    // OpMode-related variables
    protected final OpMode opMode;
    protected Telemetry telemetry;
    protected Gamepad gamepad;

    // Timer for loop execution time
    protected final ElapsedTime loopTime = new ElapsedTime();

    // Devices
    @Device("arm") protected DcMotorEx arm;
    @Device("wrist") protected Servo wrist;
    @Device("left_release") protected Servo leftRelease;
    @Device("right_release") protected Servo rightRelease;

    // Arm state variables
    protected String currentStatus;
    protected int latestArmPosition; // in encoder ticks
    protected boolean buttonWasPressed;

    /**
     * Constructor for BasicArm.
     *
     * @param opMode The OpMode in which the arm will operate.
     */
    public BaseArm(OpMode opMode) {
        this.opMode = opMode;
        this.gamepad = opMode.gamepad2;
        this.telemetry = opMode.telemetry;
    }

    /**
     * Initialize the arm subassembly.
     */
    public void init() {
        // Set up initial status and configurations
        currentStatus = "initializing";

        // Arm motor configuration
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS);

        // Wrist servo configuration
        wrist.scaleRange(0.25, 0.78); // 53% of 300-degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        // Left release servo configuration
        leftRelease.scaleRange(0.15, 0.40); // 22.5% of 300-degree range
        leftRelease.setDirection(Servo.Direction.FORWARD);

        // Right release servo configuration
        rightRelease.scaleRange(0.6, 1); // 22.5% of 300-degree range
        rightRelease.setDirection(Servo.Direction.REVERSE);

        // Initialize arm position and button state
        latestArmPosition = arm.getCurrentPosition();
        buttonWasPressed = false;

        // Display initialization completion message
        telemetry.addData(">", "Arm Subassembly Ready");
    }

    /**
     * Abstract method for the arm subassembly loop.
     */
    public abstract void loop();

    /**
     * Display relevant telemetry information.
     */
    public void telemetry() {
        telemetry.addData("Arm", "");
        telemetry.addData("status", currentStatus);
        telemetry.addData("run time (seconds)", opMode.getRuntime());
        telemetry.addData("loop time (milliseconds)", loopTime.milliseconds());
        telemetry.addData("arm mode", arm.getMode());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("arm current (amps)", arm.getCurrent(CurrentUnit.AMPS) + " out of : " + ARM_OVERCURRENT_THRESHOLD);
        telemetry.addLine();
    }


    public void setCurrentStatus(String status) { currentStatus = status; }
    public String getCurrentStatus() { return currentStatus; }

    /**
     * Move the arm to the last known position.
     */
    protected void brakeArm() {
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Check for overcurrent condition and take appropriate action.
     */
    protected void handleOvercurrentProtection() {
        if (arm.isOverCurrent()) {
            // Display warning message
            telemetry.addData("WARNING", "arm motor is overcurrent, reduce load or the arm may break");

            // Emergency stop if overcurrent is severe
            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                opMode.requestOpModeStop();
            } else {
                // Reset arm position and set mode to RUN_TO_POSITION
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else {
            // Set mode to RUN_USING_ENCODER if no overcurrent
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Control the intake servos based on gamepad input.
     */
    protected void handlePixelDroppers() {
        // Left release servo control
        if (gamepad.left_bumper) { // Open
            leftRelease.setPosition(1);
        } else if (gamepad.left_trigger > 0.2) { // Close
            leftRelease.setPosition(0);
        }
        // Right release servo control
        if (gamepad.right_bumper) { // Open
            rightRelease.setPosition(1);
        } else if (gamepad.right_trigger > 0.2) { // Close
            rightRelease.setPosition(0);
        }
    }
}