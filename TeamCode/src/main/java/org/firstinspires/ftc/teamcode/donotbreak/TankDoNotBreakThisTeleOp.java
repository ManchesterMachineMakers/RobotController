package org.firstinspires.ftc.teamcode.donotbreak;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tank Do Not Break This TeleOp", group = "do not break")
public class TankDoNotBreakThisTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize two motors
        DcMotor leftDrive = hardwareMap.dcMotor.get("left");
        DcMotor rightDrive = hardwareMap.dcMotor.get("right");

        configMotor(leftDrive, DcMotorSimple.Direction.REVERSE);
//        configMotor(rightDrive, DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(opModeIsActive()) {
            while (opModeIsActive()) {
                double leftY = -gamepad1.left_stick_y;
                double rightX = gamepad1.right_stick_x;

                // Calculate motor powers
                double leftPower = leftY + rightX;
                double rightPower = leftY - rightX;

                // Set motor powers
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
            }
        }
    }

    // Configures input motor to inputted direction
    public void configMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }
}