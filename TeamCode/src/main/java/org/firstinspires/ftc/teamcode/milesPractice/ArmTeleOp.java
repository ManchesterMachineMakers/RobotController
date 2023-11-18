package org.firstinspires.ftc.teamcode.milesPractice;

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

@TeleOp(name = "Semi-Auto Arm TeleOp")
public class ArmTeleOp extends LinearOpMode {

    // Math constants for getWristAndArmPositions(), better to declare once than every time it is called
    public static final double
            THETA = Math.PI / 3,
            GAMMA = Math.atan(0.0565371024735),
            D1 = 220,
            D2 = 88.9,
            L2 = 67.88,
            L3 = 59.5,
            H = 287.75,
            R = 336;

    // Configuration:
    // Amount to increment when using left or right d-pad. Allows for easier configuring
    public static final int
            PIXEL_LARGE_INCREMENT = 4,
            PIXEL_UPPER_LIMIT = 7,
            ARM_POSITION_FOR_FLOOR = 0, // TODO: find encoder value for this
            ARM_STOW_POSITION = 100; // TODO: find encoder value for this

    public static final double
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
        arm.setTargetPositionTolerance(50);

        // Configure servos
        leftRelease.scaleRange(0.175, 0.4); // 22.5% of 280 degree range
        leftRelease.setDirection(Servo.Direction.FORWARD);
        rightRelease.scaleRange(0.6, 0.825); // 22.5% of 280 degree range
        rightRelease.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0.25, 0.78); // 53% of 280 degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        // Declare once, rather than every opLoop
        double  r, robotAngle, v1, v2, v3, v4,
                targetWristPosition = wrist.getPosition();
        float rightX;
        int pixelStack = -1, targetArmPosition = 0, i = 0;
        boolean buttonWasPressed = false, armIsStowed = false;

        String  leftReleaseStatus = "unknown",
                rightReleaseStatus = "unknown";

        waitForStart();

        if (opModeIsActive()) {
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
                arm.setPower(Math.pow(ARM_POWER, (1 / gamepad2.left_stick_y))); //  Arm power to the reciprocal of gamepad y


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

                // Counter for pixel layer. Uses d-pad
                if (gamepad2.dpad_up && !buttonWasPressed) {
                    pixelStack++;
                } else if (gamepad2.dpad_down && !buttonWasPressed) {
                    pixelStack--;
                } else if (gamepad2.dpad_right && !buttonWasPressed) {
                pixelStack += PIXEL_LARGE_INCREMENT;
                } else if (gamepad2.dpad_left && !buttonWasPressed) {
                    pixelStack -= PIXEL_LARGE_INCREMENT;
                }

                // Makes sure pixel stack cannot go above 7 or below -1
                if (pixelStack > PIXEL_UPPER_LIMIT) {
                    pixelStack = PIXEL_UPPER_LIMIT;
                } else if (pixelStack < -1) {
                    pixelStack = -1;
                }
//
//                // Finds positioning for the arm and wrist servo so it lines up with the easel
//                if (!armIsStowed) {
//                    if (pixelStack != -1) {
//                        targetArmPosition = (int) (getArmAndWristPosition(pixelStack)[0] * ENCODER_RESOLUTION); // convert radians to encoder pulses
//                        targetWristPosition = getArmAndWristPosition(pixelStack)[1] * 0.53 * (7 / 9); // convert radians to servo range
//                    } else {
//                        targetArmPosition = ARM_POSITION_FOR_FLOOR;
//                    }
//                }
//
//                arm.setTargetPosition(targetArmPosition);
//                arm.setPower(ARM_SPEED);
//                wrist.setPosition(targetWristPosition);

                // Reset encoders:
                if (gamepad2.b) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else {
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                // Stow arm:
                if (gamepad2.x && !buttonWasPressed) {
                    if (armIsStowed) {
                        armIsStowed = false;
                    } else {
                        armIsStowed = true;
                        arm.setTargetPosition(ARM_STOW_POSITION);
                        wrist.setPosition(WRIST_STOW_POSITION);
                    }
                }

                buttonWasPressed = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.x;

                // Telemetry:
                // Used for easier debugging of code:
                telemetry.addData("Debug Info", "");
                telemetry.addData("Loop time (milliseconds)", loopTime.milliseconds());
                telemetry.addData("Button was pressed", buttonWasPressed);
                telemetry.addData("Arm position", arm.getCurrentPosition());
                telemetry.addData("Wrist position", wrist.getPosition());
                telemetry.addData("Target arm position", targetArmPosition);
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
    void configDriveMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    // Some Isaac wizzadry
    // See math: https://drive.google.com/file/d/1ADeKl-3EPOc8nBHZwGThREwBQAEdIAJ9/view
//    double findServoPosition(int armPosition) {
//
//    }
}