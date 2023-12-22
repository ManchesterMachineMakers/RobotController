package org.firstinspires.ftc.teamcode.subassemblies.miles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.manchestermachinemakers.easyop.Subassembly;

/**
 * Drivebase class representing the robot's drivetrain.
 */
public class Drivebase implements Subassembly {

    // OpMode-related variables
    private final OpMode opMode;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private final HardwareMap hardwareMap;

    // Timer for tracking loop execution time
    public ElapsedTime loopTime = new ElapsedTime();

    // Motor instances for the drivebase
    public DcMotor leftFront, rightFront, leftRear, rightRear;

    // Current status of the drivebase
    private String currentStatus = "unknown";

    // Constructor
    public Drivebase(OpMode opMode) {
        this.opMode = opMode;
        this.gamepad = opMode.gamepad1;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
    }

    /**
     * Initializes the drivebase motors and sets their configurations.
     */
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        gamepad = opMode.gamepad1;
        telemetry = opMode.telemetry;

        currentStatus = "initializing";

        // Configure each motor
        configMotor(leftFront, DcMotorSimple.Direction.REVERSE);
        configMotor(rightFront, DcMotorSimple.Direction.FORWARD);
        configMotor(leftRear, DcMotorSimple.Direction.FORWARD);
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE);

        currentStatus = "initialized; waiting for start";
        telemetry.addData(">", "Drive Base Ready.");
    }

    /**
     * Main loop for controlling robot motion based on gamepad input.
     */
    public void loop() {
        currentStatus = "looping";
        loopTime.reset();

        // Variables for robot motion calculations
        double r = Math.hypot(gamepad.left_stick_x, -gamepad.left_stick_y);
        double robotAngle = Math.atan2(-gamepad.left_stick_y, -gamepad.left_stick_x) - Math.PI / 4;
        float rightX = gamepad.right_stick_x;

        // Calculate power for each motor
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        // Set power to each motor, divided by a factor for adjustment
        leftFront.setPower(v1 / 1.2);
        rightFront.setPower(v2 / 1.2);
        leftRear.setPower(v3 / 1.2);
        rightRear.setPower(v4 / 1.2);
    }

    /**
     * Displays relevant telemetry information.
     */
    public void telemetry() {
        telemetry.addData("Drive Base", "");
        telemetry.addData("status", currentStatus);
        telemetry.addData("run time (seconds)", (int) opMode.getRuntime());
        telemetry.addData("loop time (milliseconds)", (int) loopTime.milliseconds());
        telemetry.addLine();
    }

    public void setCurrentStatus(String status) { currentStatus = status; }
    public String getCurrentStatus() { return currentStatus; }

    /**
     * Configures a motor with specified settings.
     */
    private void configMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
    }
}
