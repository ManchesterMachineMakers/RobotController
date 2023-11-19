package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Controls:
    Gamepad 1:
        Left Stick:
            Forward: moves robot forward
            Backward: moves robot backward
            Left: strafe right (note: needs to be inverted)
            Right: strafe left (note: needs to be inverted)
        Right Stick:
            Left: turn robot counter-clockwise
            Right: turn robot clockwise
    Gamepad 2:
        Left Bumper: open left release
        Left Trigger: close left release
        Right Bumper: open right release
        Right Trigger: close left release
        D-Pad:
            Up: increment pixel layer by one
            Down: decrement pixel layer by one
            Right: increment pixel layer by specified amount (5)
            Left: decrement pixel layer by specified amount (5)
        B: reset encoder
        X: stow arm
 */

@TeleOp(name = "OLD Semi-Auto Arm TeleOp")
public class ArmTeleOp extends LinearOpMode {

    // Math constants for getWristAndArmPositions(), better to declare once than every time it is called
    private static final double
            THETA = Math.PI / 3,
            GAMMA = Math.atan(0.0565371024735),
            D1 = 220,
            D2 = 88.9,
            L2 = 67.88,
            L3 = 59.5,
            H = 287.75,
            R = 336;

    public static int latestArmPosition = 0;

    // Configuration:
    // Amount to increment when using left or right d-pad. Allows for easier configuring
    private static final int
            PIXEL_LARGE_INCREMENT = 4,
            PIXEL_UPPER_LIMIT = 7,
            ARM_POSITION_FOR_FLOOR = 0, // TODO: find encoder value for this
            ARM_STOW_POSITION = 100; // TODO: find encoder value for this

    private static final double
            ARM_POWER = 0.2,
            WRIST_ANGLE_FOR_FLOOR = 0.38,
            WRIST_STOW_POSITION = 1,
            ENCODER_RESOLUTION = 2786.2;

    public ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize motors
        DcMotorEx leftFront = (DcMotorEx) hardwareMap.dcMotor.get("left_front"),
                rightFront = (DcMotorEx) hardwareMap.dcMotor.get("right_front"),
                leftRear = (DcMotorEx) hardwareMap.dcMotor.get("left_rear"),
                rightRear = (DcMotorEx) hardwareMap.dcMotor.get("right_rear"),
                arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");

        // Initialize servos
        Servo   wrist = hardwareMap.servo.get("wrist"), // Four-bar-linkage
                leftRelease = hardwareMap.servo.get("left_release"),
                rightRelease = hardwareMap.servo.get("right_release"); // NOTE: getPosition() does not work on this servo

        // Configuring all the drive motors. Yes, the code is repetitive, but it is less complicated, more readable, and less work
        configDriveMotor(leftFront, DcMotorSimple.Direction.REVERSE);
        configDriveMotor(rightFront, DcMotorSimple.Direction.FORWARD);
        configDriveMotor(leftRear, DcMotorSimple.Direction.FORWARD);
        configDriveMotor(rightRear, DcMotorSimple.Direction.REVERSE);

        // Configure arm
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setTargetPositionTolerance(50);

        // Configure servos
        leftRelease.scaleRange(0.175, 0.4); // 22.5% of 280 degree range
        leftRelease.setDirection(Servo.Direction.FORWARD);
        rightRelease.scaleRange(0.6, 0.825); // 22.5% of 280 degree range
        rightRelease.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0.25, 0.78); // 53% of 280 degree range
        wrist.setDirection(Servo.Direction.REVERSE);

        // Declare once, rather than every opLoop
        double  r, robotAngle, v1, v2, v3, v4,
                targetWristPosition = wrist.getPosition();
        float rightX;
        int pixelStack = -1, i = 0;
        boolean buttonWasPressed = false, armIsStowed = false;

        String  leftReleaseStatus = "unknown",
                rightReleaseStatus = "unknown";

        waitForStart();

        if (opModeIsActive()) {
            wrist.setPosition(0);
            sleep(1000);
            wrist.setPosition(1);
            sleep(1000);
            while (opModeIsActive()) {
                // Keep track of time spent in each loop, good for debugging
                loopTime.reset();

                // Drive base control (Power Curve)
                r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                rightX = gamepad1.right_stick_x;
                v1 = r * Math.cos(robotAngle) + rightX;
                v2 = r * Math.sin(robotAngle) - rightX;
                v3 = r * Math.sin(robotAngle) + rightX;
                v4 = r * Math.cos(robotAngle) - rightX;
                leftFront.setPower(v1 / 1.2);
                rightFront.setPower(v2 / 1.2);
                leftRear.setPower(v3 / 1.2);
                rightRear.setPower(v4 / 1.2);

                // Arm movement
                if (gamepad2.left_stick_y != 0.0) {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setPower(-gamepad2.left_stick_y / 5);
                    latestArmPosition = arm.getCurrentPosition();
                } else {
                    brake(arm);
                }

                // Wrist movement
                targetWristPosition = findWristPosition(arm.getCurrentPosition());
                wrist.setPosition(targetWristPosition);

                // Pixel release mechanism (brush)
                if (gamepad2.left_bumper) { // Open
                    leftRelease.setPosition(1);
                    leftReleaseStatus = "open";
                } else if (gamepad2.left_trigger > 0.2) { // Close
                    leftRelease.setPosition(0);
                    leftReleaseStatus = "closed";
                }

                if (gamepad2.right_bumper) { // Open
                    rightRelease.setPosition(1);
                    rightReleaseStatus = "open";
                } else if (gamepad2.right_trigger > 0.2) { // Close
                    rightRelease.setPosition(0);
                    rightReleaseStatus = "closed";
                }

                // Makes sure pixel stack cannot go above 7 or below -1
                if (pixelStack > PIXEL_UPPER_LIMIT) {
                    pixelStack = PIXEL_UPPER_LIMIT;
                } else if (pixelStack < -1) {
                    pixelStack = -1;
                }

                // Reset encoders:
                if (gamepad2.b) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                // Stow arm:
                if (gamepad2.x && !buttonWasPressed) {
                    if (armIsStowed) {
                        armIsStowed = false;
                    } else {
                        armIsStowed = true;
//                        arm.setTargetPosition(ARM_STOW_POSITION);
                        wrist.setPosition(WRIST_STOW_POSITION);
                    }
                }

                buttonWasPressed = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.x;

                // Telemetry:
                // Used for easier debugging of code:
                telemetry.addData("Debug Info", "");
                telemetry.addData("Loop time (milliseconds)", loopTime.milliseconds());
//                telemetry.addData("Button was pressed", buttonWasPressed);
                telemetry.addData("Arm mode", arm.getMode());
                telemetry.addData("Arm position", arm.getCurrentPosition());
                telemetry.addData("Target arm position", arm.getTargetPosition());
                telemetry.addData("Latest arm position", latestArmPosition);
                telemetry.addData("Arm position discrepency", arm.getCurrentPosition() - arm.getTargetPosition());
                telemetry.addData("Wrist position", wrist.getPosition());
                telemetry.addData("Target wrist position", targetWristPosition);
                telemetry.addLine();
                // Assists driver:
                telemetry.addData("Current pixel layer", pixelStack);
                telemetry.addData("Left release position", leftReleaseStatus);
                telemetry.addData("Right release position", rightReleaseStatus);
                telemetry.update();
            }
        }
    }

    // Configures a DcMotor for driving
    private void configDriveMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    private void brake(DcMotorEx arm) {
        arm.setPower(0.2);
        arm.setTargetPosition(latestArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//     See math: https://drive.google.com/file/d/1ADeKl-3EPOc8nBHZwGThREwBQAEdIAJ9/view
    private double findWristPosition(int armPosition) {
        // THETA is the angle of wrist (goal)
        // GAMMA is the angle of
        double armPositionInRadians = armPosition / ENCODER_RESOLUTION * (2 * Math.PI);
        double servoPositionInRadians = Math.PI/2.0 + THETA - GAMMA - armPositionInRadians; // replaced 90 with PI/2 (one quarter of rotation)
        double targetServoPosition = servoPositionInRadians * 0.53 * (7.0/9.0);
        telemetry.addData("Target servo position", targetServoPosition);
        telemetry.addData("Input encoder value", armPosition);
        return targetServoPosition;
    }

}