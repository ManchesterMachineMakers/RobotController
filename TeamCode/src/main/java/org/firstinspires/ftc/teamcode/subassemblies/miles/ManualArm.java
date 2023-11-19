package org.firstinspires.ftc.teamcode.subassemblies.miles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ManualArm {

    public Gamepad _Gamepad;
    public Telemetry _Telemetry;
    public HardwareMap _HardwareMap;
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

    public void init() {

        arm = _HardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = _HardwareMap.get(Servo.class, "left_release");
        rightRelease = _HardwareMap.get(Servo.class, "right_release");
        wrist = _HardwareMap.get(Servo.class, "wrist");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        _Telemetry.addData(">","Arm Subassembly Ready.");
    }

    public void loop() {
        loopTime.reset(); // Keep track of time spent in each loop for debugging

        // Wrist control
        wristPosition = wrist.getPosition();
        wristPosition += _Gamepad.right_stick_y / 25;
        if (wristPosition < 0) {
            wristPosition = 0;
        } else if (wristPosition > 1) {
            wristPosition = 1;
        }
        if (_Gamepad.left_stick_y != 0.0) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(-_Gamepad.left_stick_y / 5);
            latestArmPosition = arm.getCurrentPosition();
        } else {
            brakeArm();
        }

        // Wrist control that works
        if (!buttonWasPressed) {
            if (_Gamepad.dpad_up) {
                wristPosition -= 0.05;
            } else if (_Gamepad.dpad_down) {
                wristPosition += 0.05;
            } else if (_Gamepad.dpad_left) {
                wristPosition += 0.2;
            } else if (_Gamepad.dpad_right) {
                wristPosition -= 0.2;
            }
        }
        wrist.setPosition(wristPosition);

        // Pixel release mechanism (brush)
        // Left
        if (_Gamepad.left_bumper) { // Open
            leftRelease.setPosition(1);
            leftReleaseStatus = "open";
        } else if (_Gamepad.left_trigger > 0.2) { // Close
            leftRelease.setPosition(0);
            leftReleaseStatus = "closed";
        }
        // Right
        if (_Gamepad.right_bumper) { // Open
            rightRelease.setPosition(1);
            rightReleaseStatus = "open";
        } else if (_Gamepad.right_trigger > 0.2) { // Close
            rightRelease.setPosition(0);
            rightReleaseStatus = "closed";
        }

        // For detecting when a button is pressed.
        buttonWasPressed = _Gamepad.dpad_up || _Gamepad.dpad_down || _Gamepad.dpad_left || _Gamepad.dpad_right;
    }

    public void telemetry() {
        _Telemetry.addData("Manual Arm", "");
        _Telemetry.addData("Loop time (nanoseconds)", loopTime.nanoseconds());
        _Telemetry.addData("Arm mode", arm.getMode());
        _Telemetry.addData("Arm target position", arm.getTargetPosition());
        _Telemetry.addData("Arm position", arm.getCurrentPosition());
        _Telemetry.addData("Arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition())   ;
        _Telemetry.addData("Wrist position", wrist.getPosition());
        _Telemetry.addData("Left release position", leftReleaseStatus);
        _Telemetry.addData("Right release position", rightReleaseStatus);
        _Telemetry.addLine();
    }

    private void brakeArm() {
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
