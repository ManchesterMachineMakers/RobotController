package org.firstinspires.ftc.teamcode.subassemblies.miles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.manchestermachinemakers.easyop.Device;
import org.manchestermachinemakers.easyop.Linear;

// Comments courtesy of ChatGPT
public class Drivebase extends Linear {

    private final OpMode opMode;

    public Drivebase(OpMode opMode) { this.opMode = opMode; }

    // References to hardware map, gamepad, and telemetry
    private Gamepad gamepad;
    private Telemetry telemetry;

    // Timer for tracking loop execution time
    public ElapsedTime loopTime = new ElapsedTime();

    // Motor instances for the drivebase
    @Device("left_front") public DcMotor leftFront;
    @Device("right_front") public DcMotor rightFront;
    @Device("left_rear") public DcMotor leftRear;
    @Device("right_rear") public DcMotor rightRear;

    public String currentStatus = "unknown";
    public double runTime = 0;

    // Initializes the drivebase motors and sets their configurations
    @Override public void opInit() {
        gamepad = opMode.gamepad1;
        telemetry = opMode.telemetry;

        currentStatus = "initializing";

        configMotor(leftFront, DcMotorSimple.Direction.REVERSE);
        configMotor(rightFront, DcMotorSimple.Direction.FORWARD);
        configMotor(leftRear, DcMotorSimple.Direction.FORWARD);
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE);

        currentStatus = "initialized; waiting for start";
        telemetry.addData(">", "Drive Base Ready.");
    }

    // Main loop for controlling robot motion based on gamepad input
    @Override public void opLoop() {
        currentStatus = "looping";
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

    // Displays relevant telemetry information
    public void telemetry() {
        telemetry.addData("Drive Base", "");
        telemetry.addData("status", currentStatus);
        telemetry.addData("run time", (int) runTime);
        telemetry.addData("loop time (milliseconds)", (int) loopTime.milliseconds());
        telemetry.addLine();
    }

    // Configures a motor with specified settings
    private void configMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
    }

}