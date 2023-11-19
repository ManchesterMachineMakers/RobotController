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

public class SemiAutoArm {

    public Gamepad _gamepad;
    public Telemetry _telemetry;
    public HardwareMap _hardwareMap;

    // constants
    public static final double ARM_ENCODER_RESOLUTION = 2786.2; // in pulses per rotation
    public static final double ARM_POWER = 0.4;
    public static final double ARM_OVERCURRENT_THRESHOLD = 4;
    // position presets
    public static final int ARM_HORIZONTAL_POSITION = 1550;
    public static final int ARM_WINCH_POSITION = 0; // TODO: get value for this
    public static final double WRIST_WINCH_POSITION = 0; // TODO: get value for this
    // math constants
    public static final double GAMMA = Math.atan(16.0/283.0) * (180/Math.PI);

    public ElapsedTime loopTime = new ElapsedTime();

    public DcMotorEx arm;
    public Servo leftRelease;
    public Servo rightRelease;
    public Servo wrist;

    public int latestArmPosition = ARM_HORIZONTAL_POSITION;
    public double theta = 60;
    public String leftReleaseStatus = "unknown";
    public String rightReleaseStatus = "unknown";
    public String wristAlignment = "easel";
    public boolean needsStop = false;
    
    public void init() {
        
        arm = _hardwareMap.get(DcMotorEx.class, "arm");
        leftRelease = _hardwareMap.get(Servo.class, "left_release");
        rightRelease = _hardwareMap.get(Servo.class, "right_release");
        wrist = _hardwareMap.get(Servo.class, "wrist");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setCurrentAlert(ARM_OVERCURRENT_THRESHOLD, CurrentUnit.AMPS); // threshold for current usage alert

        leftRelease.scaleRange(0.15, 0.40); // 22.5% of 300 degree range, open = max
        leftRelease.setDirection(Servo.Direction.FORWARD);

        rightRelease.scaleRange(0.6, 1); // 22.5% of 300 degree range, open = min // 8.75
        rightRelease.setDirection(Servo.Direction.REVERSE);

        wrist.scaleRange(0.25, 0.78); // 53% of 300 degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        _telemetry.addData(">", "Semi-Auto Arm Ready.");
    }

    public void loop() {

        loopTime.reset();

        protectArmIfOverCurrent();

        // arm movement
        if (!arm.isOverCurrent()) {
            if (_gamepad.left_stick_y != 0) {
                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                arm.setPower(_gamepad.left_stick_y * ARM_POWER);
                latestArmPosition = arm.getCurrentPosition();
            } else {
                brakeArm();
            }
        }

        // wrist movement
        wrist.setPosition(findWristPosition());

        // intake
        if (_gamepad.left_bumper) {
            leftRelease.setPosition(1);
            leftReleaseStatus = "open";
        } else if (_gamepad.left_trigger > 0.2) {
            leftRelease.setPosition(0);
            leftReleaseStatus = "closed";
        }
        if (_gamepad.right_bumper) {
            rightRelease.setPosition(1);
            rightReleaseStatus = "open";
        } else if (_gamepad.right_trigger > 0.2) {
            rightRelease.setPosition(0);
            rightReleaseStatus = "closed";
        }

        // reset arm position
        if (_gamepad.x) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (_gamepad.a) {
            theta = 120;
            wristAlignment = "floor";
        } else if (_gamepad.y) {
            theta = 60;
            wristAlignment = "easel";
        }
    }

    public void telemetry() {

        // a lot of Telemetry
        _telemetry.addData("Semi-Automatic Arm", "");
        _telemetry.addData("loop time (milliseconds)", loopTime.milliseconds());
        _telemetry.addData("is arm busy", arm.isBusy());
        _telemetry.addData("arm mode", arm.getMode());
        _telemetry.addData("arm position", arm.getCurrentPosition());
        _telemetry.addData("arm target position", arm.getTargetPosition());
        _telemetry.addData("arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition());
        _telemetry.addData("wrist position", wrist.getPosition());
        _telemetry.addData("wrist alignment", wristAlignment);
        _telemetry.addData("left release status", leftReleaseStatus);
        _telemetry.addData("right release status", rightReleaseStatus);
        _telemetry.addData("arm motor current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        _telemetry.addData("is arm over current", arm.isOverCurrent());
        _telemetry.addLine();
    }

    private double findWristPosition() {

        // THETA is the angle of wrist (goal)
        // GAMMA is the angle of
        double armAngle = 360 * arm.getCurrentPosition() / ARM_ENCODER_RESOLUTION; //Described as beta in documentation
        double servoAngle = 90 + theta - GAMMA - armAngle; // Described as alpha in documentation
        return (servoAngle - 90) / (0.53 * 300) - (0.5 * 0.53);
                //servoAngle * 0.53 * (7.0/9.0); // target servo position
    }

    private void brakeArm() {
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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