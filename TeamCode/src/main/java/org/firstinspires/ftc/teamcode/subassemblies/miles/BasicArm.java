package org.firstinspires.ftc.teamcode.subassemblies.miles;

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

public class BasicArm implements Subassembly {

    protected static final double ARM_ENCODER_RES = 2786.2; // PPR
    protected static final double ARM_SPEED = 0.4;
    protected static final double ARM_OVERCURRENT_THRESHOLD = 4; // Amps

    protected final OpMode opMode;
    protected Gamepad gamepad;
    protected Telemetry telemetry;

    protected final ElapsedTime loopTime = new ElapsedTime();

    @Device("arm") protected DcMotorEx arm;
    @Device("wrist") protected Servo wrist;
    @Device("left_release") protected Servo leftRelease;
    @Device("right_release") protected Servo rightRelease;

    public String currentStatus;
    protected int latestArmPosition; // in encoder ticks
    protected boolean buttonWasPressed;

    public BasicArm(OpMode opMode) {
        this.opMode = opMode;
        this.gamepad = opMode.gamepad2;
        this.telemetry = opMode.telemetry;
    }

    public void init() {
        currentStatus = "initializing";

        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS);

        wrist.scaleRange(0.25, 0.78); // 53% of 300-degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        leftRelease.scaleRange(0.15, 0.40); // 22.5% of 300-degree range
        leftRelease.setDirection(Servo.Direction.FORWARD);

        rightRelease.scaleRange(0.6, 1); // 22.5% of 300-degree range
        rightRelease.setDirection(Servo.Direction.REVERSE);

        latestArmPosition = arm.getCurrentPosition();
        buttonWasPressed = false;

        telemetry.addData(">", "Arm Subassembly Ready");
    }

    public void loop() {
        loopTime.reset();
        currentStatus = "looping";

        if (!arm.isOverCurrent()) {
            if (gamepad.left_stick_y != 0) {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-gamepad.left_stick_y * ARM_SPEED);
                latestArmPosition = arm.getCurrentPosition();
            } else {
                brakeArm();
            }
        }

        controlIntakeServos();

        buttonWasPressed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right;
    }

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

    public void overcurrentProtection() {

        if (arm.isOverCurrent()) {

            telemetry.addData("WARNING", "arm motor is overcurrent, reduce load or the arm may break");

            if (arm.getCurrent(CurrentUnit.AMPS) > ARM_OVERCURRENT_THRESHOLD * 1.4) {
                opMode.requestOpModeStop();
            } else {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    protected void brakeArm() {
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void controlIntakeServos() {
        if (gamepad.left_bumper) { // Open
            leftRelease.setPosition(1);
        } else if (gamepad.left_trigger > 0.2) { // Close
            leftRelease.setPosition(0);
        }
        // Right
        if (gamepad.right_bumper) { // Open
            rightRelease.setPosition(1);
        } else if (gamepad.right_trigger > 0.2) { // Close
            rightRelease.setPosition(0);
        }
    }
}
