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

public class ManualArm {

    public static final double ARM_SPEED = 0.5;
    public static final double ARM_OVERCURRENT_THRESHOLD = 5;

    public Gamepad _gamepad;
    public Telemetry _telemetry;
    public HardwareMap _hardwareMap;
    public ElapsedTime loopTime = new ElapsedTime();

    public DcMotorEx arm;

    public Servo leftRelease;
    public Servo rightRelease;
    public Servo wrist;

    public String leftReleaseStatus;
    public String rightReleaseStatus;
    public int latestArmPosition;
    public double wristPosition;
    public boolean buttonWasPressed;
    public boolean needsStop;

    public void init() {

        arm = _hardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = _hardwareMap.get(Servo.class, "left_release");
        rightRelease = _hardwareMap.get(Servo.class, "right_release");
        wrist = _hardwareMap.get(Servo.class, "wrist");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS);

        leftRelease.scaleRange(0.175, 0.4); // 7/40 radians
        leftRelease.setDirection(Servo.Direction.FORWARD);
        rightRelease.scaleRange(0.6, 0.825); // 7/40 radians
        rightRelease.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0.25, 0.78); // 14/45 radians
        wrist.setDirection(Servo.Direction.FORWARD);

        leftReleaseStatus = "unknown";
        rightReleaseStatus = "unknown";
        latestArmPosition = 0;
        buttonWasPressed = false;

        _telemetry.addData(">","Arm Subassembly Ready.");
    }

    public void loop() {
        loopTime.reset(); // Keep track of time spent in each loop for debugging

        protectArmIfOverCurrent();

        // Wrist control
        wristPosition = wrist.getPosition();
        wristPosition += _gamepad.right_stick_y / 25;
        if (wristPosition < 0) {
            wristPosition = 0;
        } else if (wristPosition > 1) {
            wristPosition = 1;
        }
        if (_gamepad.left_stick_y != 0.0) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(-_gamepad.left_stick_y * ARM_SPEED);
            latestArmPosition = arm.getCurrentPosition();
        } else {
            brakeArm();
        }

        // Wrist control that works
        if (!buttonWasPressed) {
            if (_gamepad.dpad_up) {
                wristPosition -= 0.05;
            } else if (_gamepad.dpad_down) {
                wristPosition += 0.05;
            } else if (_gamepad.dpad_left) {
                wristPosition += 0.2;
            } else if (_gamepad.dpad_right) {
                wristPosition -= 0.2;
            }
        }
        wrist.setPosition(wristPosition);

        if (_gamepad.b) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Pixel release mechanism (brush)
        // Left
        if (_gamepad.left_bumper) { // Open
            leftRelease.setPosition(1);
            leftReleaseStatus = "open";
        } else if (_gamepad.left_trigger > 0.2) { // Close
            leftRelease.setPosition(0);
            leftReleaseStatus = "closed";
        }
        // Right
        if (_gamepad.right_bumper) { // Open
            rightRelease.setPosition(1);
            rightReleaseStatus = "open";
        } else if (_gamepad.right_trigger > 0.2) { // Close
            rightRelease.setPosition(0);
            rightReleaseStatus = "closed";
        }

        // For detecting when a button is pressed.
        buttonWasPressed = _gamepad.dpad_up || _gamepad.dpad_down || _gamepad.dpad_left || _gamepad.dpad_right;
    }

    public void telemetry() {
        _telemetry.addData("Manual Arm", "");
        _telemetry.addData("loop time (nanoseconds)", loopTime.nanoseconds());
        _telemetry.addData("arm mode", arm.getMode());
        _telemetry.addData("arm target position", arm.getTargetPosition());
        _telemetry.addData("arm position", arm.getCurrentPosition());
        _telemetry.addData("arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition())   ;
        _telemetry.addData("wrist position", wrist.getPosition());
        _telemetry.addData("left release position", leftReleaseStatus);
        _telemetry.addData("right release position", rightReleaseStatus);
        _telemetry.addData("arm current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        _telemetry.addLine();
    }

    private void brakeArm() {
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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
