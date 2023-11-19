package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "OLD Manual Arm TeleOp")
public class ManualArmTeleOp extends LinearOpMode {

    // Timer, good for debugging
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        DcMotorEx
                leftFront = (DcMotorEx) hardwareMap.dcMotor.get("left_front"),
                rightFront = (DcMotorEx) hardwareMap.dcMotor.get("right_front"),
                leftRear = (DcMotorEx) hardwareMap.dcMotor.get("left_rear"),
                rightRear = (DcMotorEx) hardwareMap.dcMotor.get("right_rear"),
                arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");

        Servo
                wrist = hardwareMap.servo.get("wrist"), // Four-bar-linkage
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
        double r, robotAngle, v1, v2, v3, v4, wristPosition;
        float rightX;
        int pixelStack = -1, latestArmPosition = 0;
        boolean buttonWasPressed = false;

        String  leftReleaseStatus = "unknown",
                rightReleaseStatus = "unknown";

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                loopTime.reset(); // Keep track of time spent in each loop for debugging

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

                // Wrist control
                wristPosition = wrist.getPosition();
                wristPosition += gamepad2.right_stick_y / 25;
                if (wristPosition < 0) {
                    wristPosition = 0;
                } else if (wristPosition > 1) {
                    wristPosition = 1;
                }
                if (gamepad2.left_stick_y != 0.0) {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setPower(-gamepad2.left_stick_y / 5);
                    latestArmPosition = arm.getCurrentPosition();
                } else {
                    brake(arm, latestArmPosition);
                }

                // Wrist control that works
                if (!buttonWasPressed) {
                    if (gamepad2.dpad_up) {
                        wristPosition -= 0.05;
                    } else if (gamepad2.dpad_down) {
                        wristPosition += 0.05;
                    } else if (gamepad2.dpad_left) {
                        wristPosition += 0.2;
                    } else if (gamepad2.dpad_right) {
                        wristPosition -= 0.2;
                    }
                }
                wrist.setPosition(wristPosition);

                // Pixel release mechanism (brush)
                // Left
                if (gamepad2.left_bumper) { // Open
                    leftRelease.setPosition(1);
                    leftReleaseStatus = "open";
                } else if (gamepad2.left_trigger > 0.2) { // Close
                    leftRelease.setPosition(0);
                    leftReleaseStatus = "closed";
                }
                // Right
                if (gamepad2.right_bumper) { // Open
                    rightRelease.setPosition(1);
                    rightReleaseStatus = "open";
                } else if (gamepad2.right_trigger > 0.2) { // Close
                    rightRelease.setPosition(0);
                    rightReleaseStatus = "closed";
                }

                // For detecting when a button is pressed.
                buttonWasPressed = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;

                // Telemetry:
                // Used for easier debugging of code:
                telemetry.addData("Debug Info", "");
                telemetry.addData("Loop time (nanoseconds)", loopTime.nanoseconds());
                telemetry.addData("Arm mode", arm.getMode());
                telemetry.addData("Arm target position", arm.getTargetPosition());
                telemetry.addData("Arm position", arm.getCurrentPosition());
                telemetry.addData("Arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition())   ;
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
    private void configDriveMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

    // Makes a motor actually brake (ZeroPowerBehavior doesn't give enough power)
    private void brake(DcMotorEx motor, int latestPosition) {
        motor.setTargetPosition(latestPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}