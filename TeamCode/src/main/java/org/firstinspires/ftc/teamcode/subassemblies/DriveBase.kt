package org.firstinspires.ftc.teamcode.subassemblies;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.contracts.Controllable;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.manchestermachinemakers.easyop.Device;
import org.manchestermachinemakers.easyop.Subassembly;

import java.util.HashMap;

public class DriveBase implements Subassembly, Controllable {
    @Device
    public DcMotor leftFront;
    @Device
    public DcMotor rightFront;
    @Device
    public DcMotor leftRear;
    @Device
    public DcMotor rightRear;

    private void configMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Runs motor without distance tracking
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brakes motor when stopping
        motor.setDirection(direction); // Sets motors to their specified direction
    }

    @Override
    public void customInit(HardwareMap hardwareMap) {
        configMotor(leftFront, DcMotorSimple.Direction.FORWARD);
        configMotor(rightFront, DcMotorSimple.Direction.REVERSE);
        configMotor(leftRear, DcMotorSimple.Direction.FORWARD);
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Taken from the RobotAutoDriveToAprilTagOmni example (2023) Move robot according to desired
     * axes motions <p> Positive X is forward <p> Positive Y is strafe left <p> Positive Yaw is
     * counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftBackPower);
        rightRear.setPower(rightBackPower);
    }


    public void controller(GamepadManager gamepad) {
        double r = hypot(gamepad.getGamepad().left_stick_x, -gamepad.getGamepad().left_stick_y);
        double robotAngle =
                atan2(-gamepad.getGamepad().left_stick_y, gamepad.getGamepad().left_stick_x) - Math.PI / 4;
        double rightX = gamepad.getGamepad().right_stick_x;
        double v1 = r * cos(robotAngle) + rightX;
        double v2 = r * sin(robotAngle) - rightX;
        double v3 = r * sin(robotAngle) + rightX;
        double v4 = r * cos(robotAngle) - rightX;
        leftFront.setPower(v1 / 1.2);
        rightFront.setPower(v2 / 1.2);
        leftRear.setPower(v3 / 1.2);
        rightRear.setPower(v4 / 1.2);
    }
}
