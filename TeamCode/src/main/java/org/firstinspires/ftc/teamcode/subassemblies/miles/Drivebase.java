package org.firstinspires.ftc.teamcode.subassemblies.miles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivebase {

    public ElapsedTime loopTime = new ElapsedTime();

    public DcMotorEx
            leftFront,
            rightFront,
            leftRear,
            rightRear;

    public double r, robotAngle, v1, v2, v3, v4;
    public float rightX;

    public HardwareMap _HardwareMap;
    public Gamepad _Gamepad;
    public Telemetry _Telemetry;

    public void init() {
        leftFront = _HardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = _HardwareMap.get(DcMotorEx.class, "right_front");
        leftRear = _HardwareMap.get(DcMotorEx.class, "left_rear");
        rightRear = _HardwareMap.get(DcMotorEx.class, "right_rear");

        configMotor(leftFront, DcMotorSimple.Direction.REVERSE);
        configMotor(rightFront, DcMotorSimple.Direction.FORWARD);
        configMotor(leftRear, DcMotorSimple.Direction.FORWARD);
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE);

        _Telemetry.addData(">", "Drive Base Ready.");
    }

    public void loop() {
        loopTime.reset();

        r = Math.hypot(_Gamepad.left_stick_x, -_Gamepad.left_stick_y);
        robotAngle = Math.atan2(-_Gamepad.left_stick_y, -_Gamepad.left_stick_x) - Math.PI / 4;
        rightX = _Gamepad.right_stick_x;
        v1 = r * Math.cos(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.sin(robotAngle) + rightX;
        v4 = r * Math.cos(robotAngle) - rightX;
        leftFront.setPower(v1 / 1.2);
        rightFront.setPower(v2 / 1.2);
        leftRear.setPower(v3 / 1.2);
        rightRear.setPower(v4 / 1.2);
    }

    // configures a motor
    private void configMotor(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
    }

    public void telemetry() {
        _Telemetry.addData("Drive Base", "");
        _Telemetry.addData("loop time (milliseconds)", loopTime.milliseconds());
        _Telemetry.addLine();
    }
}
