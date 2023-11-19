package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Do Not Break This TeleOp")
public class DoNotBreakThisTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize all motors
        DcMotor leftFront  = hardwareMap.dcMotor.get("left_front"),
                rightFront = hardwareMap.dcMotor.get("right_front"),
                leftRear   = hardwareMap.dcMotor.get("left_rear"),
                rightRear  = hardwareMap.dcMotor.get("right_rear");

        // Yes, the code is repetitive, but it has better readability and is less work
        configMotor(leftFront,  DcMotorSimple.Direction.REVERSE);
        configMotor(rightFront, DcMotorSimple.Direction.FORWARD);
        configMotor(leftRear,   DcMotorSimple.Direction.FORWARD);
        configMotor(rightRear,  DcMotorSimple.Direction.REVERSE);

        // Initialize here, rather than every loop, for greater efficiency
        float rightX;
        double  r,
                robotAngle,
                v1,
                v2,
                v3,
                v4;

        waitForStart();

        if(opModeIsActive()) {
            while (opModeIsActive()) {
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
            }
        }
    }

    // Configures input motor to inputted direction
    public void configMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Fine for TeleOp unless Semi-Auto
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

}