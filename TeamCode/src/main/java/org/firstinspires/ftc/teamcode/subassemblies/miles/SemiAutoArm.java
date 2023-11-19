package org.firstinspires.ftc.teamcode.subassemblies.miles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SemiAutoArm { //extends OpMode {

    public Gamepad _Gamepad;
    public Telemetry _Telemetry;
    public HardwareMap _HardwareMap;

    // constants
    public static final double ARM_ENCODER_RESOLUTION = 2786.2; // in pulses per rotation
    public static final double ARM_POWER = 0.2;
    // position presets
    public static final int ARM_HORIZONTAL_POSITION = 1550;
    public static final int ARM_WINCH_POSITION = 0; // TODO: get value for this
    public static final double WRIST_WINCH_POSITION = 0; // TODO: get value for this
    // math constants
    public static final double THETA = Math.PI / 3;
    public static final double GAMMA = Math.atan(0.0565371024735);

    public ElapsedTime loopTime = new ElapsedTime();

    public DcMotorEx arm;
    public Servo leftRelease;
    public Servo rightRelease;
    public Servo wrist;

    public int latestArmPosition = ARM_HORIZONTAL_POSITION;
    public String leftReleaseStatus = "unknown";
    public String rightReleaseStatus = "unknown";
    
    //@Override
    public void init() {
        
        arm = _HardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = _HardwareMap.get(Servo.class, "left_release");
        rightRelease = _HardwareMap.get(Servo.class, "right_release");
        wrist = _HardwareMap.get(Servo.class, "wrist");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftRelease.scaleRange(0.175, 0.4); // 22.5% of 280 degree range
        leftRelease.setDirection(Servo.Direction.FORWARD);

        rightRelease.scaleRange(0.6, 0.825); // 22.5% of 280 degree range
        rightRelease.setDirection(Servo.Direction.REVERSE);

        wrist.scaleRange(0.25, 0.78); // 53% of 280 degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        wrist.setPosition(1);
        arm.setTargetPosition(ARM_HORIZONTAL_POSITION);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        _Telemetry.addData(">", "Semi-Auto Arm Ready.");
    }

    public void loop() {

        loopTime.reset();

        // arm movement
        if (_Gamepad.left_stick_y != 0) {
            arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            arm.setPower(_Gamepad.left_stick_y * ARM_POWER);
            latestArmPosition = arm.getCurrentPosition();
        } else {
            brakeArm();
        }

        // wrist movement
        wrist.setPosition(findWristPosition());

        // intake
        if (_Gamepad.left_bumper) {
            leftRelease.setPosition(1);
            leftReleaseStatus = "open";
        } else if (_Gamepad.left_trigger > 0.2) {
            leftRelease.setPosition(0);
            leftReleaseStatus = "closed";
        }
        if (_Gamepad.right_bumper) {
            rightRelease.setPosition(1);
            rightReleaseStatus = "open";
        } else if (_Gamepad.right_trigger > 0.2) {
            rightRelease.setPosition(0);
            rightReleaseStatus = "closed";
        }

        // reset arm position
        if (_Gamepad.b) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private double findWristPosition() {

        // THETA is the angle of wrist (goal)
        // GAMMA is the angle of
        double armPositionInRadians = arm.getCurrentPosition() / ARM_ENCODER_RESOLUTION * (2 * Math.PI);
        double servoPositionInRadians = Math.PI/2 + THETA - GAMMA - armPositionInRadians; // replaced 90 with PI/2 (one quarter of rotation)
        return servoPositionInRadians * 0.53 * (7.0/9.0); // target servo position
    }

    private void brakeArm() {
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void telemetry() {

        // a lot of Telemetry
        _Telemetry.addData("Semi-Automatic Arm", "");
        _Telemetry.addData("loop time (milliseconds)", loopTime.milliseconds());
        _Telemetry.addData("is arm busy", arm.isBusy());
        _Telemetry.addData("arm mode", arm.getMode());
        _Telemetry.addData("arm position", arm.getCurrentPosition());
        _Telemetry.addData("arm target position", arm.getTargetPosition());
        _Telemetry.addData("arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition());
        _Telemetry.addData("wrist position", wrist.getPosition());
        _Telemetry.addData("left release status", leftReleaseStatus);
        _Telemetry.addData("right release status", rightReleaseStatus);
        _Telemetry.addLine();
    }
}