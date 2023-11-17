package org.firstinspires.ftc.teamcode.milesPractice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
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
 */

@TeleOp(name = "Arm TeleOp")
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
            PIXEL_UPPER_LIMIT = 7;

    public static final double ARM_SPEED = 0.2;

    // servo.

    /* NOTES: WE DID LOTS OF CODE TODAY 11/17/2023
        - Zach - blocks - Arm position from stow to front horizontal
        - got the motor to hold position by allowing power to remain after run to position
        - Miles - full Arm teleop
        - use the Adafruit sensor on the arm to detect desired 0 position
        - desired 0 position is horizontal, arm pointing backward, just up from stow position
        - first automation TODO: automatically position the arm and wrist to pick up from floor.
            - can do that with static values for now, until Adafruit
            - install Adafruit and average the values to get a consistent position
        - servo ranges are good
        - may need to shorten the arm by a couple of in order to fit within 18" box

     */
    public static final double
            WRIST_ANGLE_FOR_FLOOR = 0.38,
            ARM_POSITION_FOR_FLOOR = 0,
            WRIST_ANGLE_FOR_BACKDROP = 0.54;

    public ElapsedTime loopTime = new ElapsedTime();

    /**
     * E encoder position -> radians R
     * radians -> encoder position?
     * on button press X, save current E as zero (E0)
     * calculate current R using E and E0
     * calculate target E using target R and E0
     * encoder resolution is XX.X PPR (pulses per revolution)
     * target R (in radians) * (1 rev / 2PI radians) * (XX.X pulse / rev) = target E (in encoder value pulses)
     */

    @Override
    public void runOpMode() {

        // Initialize motors
        DcMotor leftFront = hardwareMap.dcMotor.get("left_front"),
                rightFront = hardwareMap.dcMotor.get("right_front"),
                leftRear = hardwareMap.dcMotor.get("left_rear"),
                rightRear = hardwareMap.dcMotor.get("right_rear"),
                arm = hardwareMap.dcMotor.get("arm");

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

        // Configure servos
        leftRelease.scaleRange(0.175, 0.4); // 7/40 radians
        leftRelease.setDirection(Servo.Direction.FORWARD);
        rightRelease.scaleRange(0.6, 0.825); // 7/40 radians
        rightRelease.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0.25, 0.78); // 14/45 radians
        wrist.setDirection(Servo.Direction.FORWARD);

        // Declare once, rather than every opLoop
        double r, robotAngle, v1, v2, v3, v4, wristPosition = 0;
        float rightX;
        int pixelStack = -1;
        boolean buttonWasPressed = false;

        String  leftReleaseStatus = "unknown",
                rightReleaseStatus = "unknown";

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Keep track of time spent in each loop
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

                wristPosition += gamepad2.right_stick_y / 25;
                if (wristPosition < 0) {
                    wristPosition = 0;
                } else if (wristPosition > 1) {
                    wristPosition = 1;
                }

                wrist.setPosition(wristPosition);
                arm.setPower(gamepad2.left_stick_y / 5); // TEMPORARY

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

                // Counter:
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

                // Write motor code here:
                /** MAKE SURE TO LIMIT MOTOR MOVEMENT */
                /**
                 * E encoder position -> radians R
                 * radians -> encoder position?
                 * on button press X, save current E as zero (E0)
                 * calculate current R using E and E0
                 * calculate target E using target R and E0
                 * encoder resolution is XX.X PPR (pulses per revolution)
                 * target R (in radians) * (1 rev / 2PI radians) * (XX.X pulse / rev) = target E (in encoder value pulses)
                 */


                buttonWasPressed = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;

                // Telemetry:
                // Used for easier debugging of code:
                telemetry.addData("Debug Info", "");
                telemetry.addData("Loop time (nanoseconds)", loopTime.nanoseconds());
                telemetry.addData("Button was pressed", buttonWasPressed);
                telemetry.addData("Arm position", arm.getCurrentPosition());
                telemetry.addData("Wrist position", wrist.getPosition());
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

    void motorToPosition(DcMotor motor, int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setPower(1);
    }

    // See math: https://drive.google.com/file/d/1ADeKl-3EPOc8nBHZwGThREwBQAEdIAJ9/view
    /**
     * @param {int} n layer of hexes we're going up to, starting at 0
     * returns an array of the target arm position (radians) and target servo position (%range?)
     */
    public double[] getArmAndWristPosition(int n) {
        double targetServoPos;
        double targetArmPos;

        targetArmPos = Math.asin(((D1 + n * D2 - L3) * Math.sin(THETA) + L2 * Math.cos(THETA) - H) / R);
        if (n <= -1) {
            targetServoPos = 0.5;
        }
        else {
            targetServoPos = 90 + THETA - GAMMA - targetArmPos;
        }

        return new double[]{targetArmPos, targetServoPos};
    }
}