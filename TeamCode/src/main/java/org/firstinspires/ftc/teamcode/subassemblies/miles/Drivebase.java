package org.firstinspires.ftc.teamcode.subassemblies.miles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Comments courtesy of ChatGPT
public class Drivebase {

    // References to hardware map, gamepad, and telemetry
    public HardwareMap hardwareMap;
    public Gamepad gamepad;
    public Telemetry telemetry;

    // Timer for tracking loop execution time
    public ElapsedTime loopTime = new ElapsedTime();

    // Motor instances for the drivebase
    public DcMotorEx
            leftFront,
            rightFront,
            leftRear,
            rightRear;

    // MAKE SURE TO DECLARE WITH VALUE
    public String currentStatus = "unknown";
    public double runTime = 0;

    // Initializes the drivebase motors and sets their configurations
    public void init() {

        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");

        configMotor(leftFront, DcMotorSimple.Direction.REVERSE);
        configMotor(rightFront, DcMotorSimple.Direction.FORWARD);
        configMotor(leftRear, DcMotorSimple.Direction.FORWARD);
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE);

        telemetry.addData(">", "Drive Base Ready.");
    }

    // Main loop for controlling robot motion based on gamepad input
    public void loop() {
        loopTime.reset();

        // Variables for robot motion calculations
        double r = Math.hypot(gamepad.left_stick_x, -gamepad.left_stick_y);
        double robotAngle = Math.atan2(-gamepad.left_stick_y, -gamepad.left_stick_x) - Math.PI / 4;
        float rightX = gamepad.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;
        leftFront.setPower(v1 / 1.2);
        rightFront.setPower(v2 / 1.2);
        leftRear.setPower(v3 / 1.2);
        rightRear.setPower(v4 / 1.2);
    }

    // Configures a motor with specific settings
    private void configMotor(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
    }

    // Displays relevant telemetry information
    public void telemetry() {
        telemetry.addData("Drive Base", "");
        telemetry.addData("status", currentStatus);
        telemetry.addData("run time", (int) runTime);
        telemetry.addData("loop time (milliseconds)", (int) loopTime.milliseconds());
        telemetry.addLine();
    }
}