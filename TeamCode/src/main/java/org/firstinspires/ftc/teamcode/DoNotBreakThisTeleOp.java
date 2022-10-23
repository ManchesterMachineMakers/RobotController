package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BasicTeleOpKt.POWER_COEFFICIENT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Do Not Break This TeleOp")
public class DoNotBreakThisTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftFront  = hardwareMap.dcMotor.get("left_front"),
                rightFront = hardwareMap.dcMotor.get("right_front"),
                leftRear   = hardwareMap.dcMotor.get("left_rear"),
                rightRear  = hardwareMap.dcMotor.get("right_rear");

        DcMotor[] motors = {leftFront, rightFront, leftRear, rightRear};
        for (DcMotor motor :
                motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotor motor : new DcMotor[]{leftFront, leftRear}) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE); // compensate for left motors
        }
        for (DcMotor motor : new DcMotor[]{rightFront, rightRear}) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD); // right motors are normal
        }

        waitForStart();

        if(opModeIsActive()) {
            while (opModeIsActive()) {
                double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                float rightX = gamepad1.right_stick_x;
                double v1 = r * Math.cos(robotAngle) + rightX;
                double v2 = r * Math.sin(robotAngle) - rightX;
                double v3 = r * Math.sin(robotAngle) + rightX;
                double v4 = r * Math.cos(robotAngle) - rightX;
                leftFront.setPower(v1 / POWER_COEFFICIENT);
                rightFront.setPower(v2 / POWER_COEFFICIENT);
                leftRear.setPower(v3 / POWER_COEFFICIENT);
                rightRear.setPower(v4 / POWER_COEFFICIENT);
            }
        }
    }
}
