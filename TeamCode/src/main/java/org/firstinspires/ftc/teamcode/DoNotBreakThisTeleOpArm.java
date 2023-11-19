package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subassemblies.Arm;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.manchestermachinemakers.easyop.Inject;
import org.manchestermachinemakers.easyop.Linear;

@TeleOp(name = "Do Not Break This TeleOp with Arm")
public class DoNotBreakThisTeleOpArm extends Linear {
    @Inject
    public Arm arm;

    @Override
    public void opBeforeLoop() {
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
        for (DcMotor motor : new DcMotor[]{leftRear, rightRear}) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE); // compensate for rear motors
        }
        for (DcMotor motor : new DcMotor[]{leftFront, rightFront}) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD); // front motors are normal
        }

        waitForStart();

        if(opModeIsActive()) {
            while (opModeIsActive()) {
                double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                float rightX = gamepad1.right_stick_x;
                double v1 = r * Math.cos(robotAngle) + rightX;
                double v2 = r * Math.sin(robotAngle) - rightX;
                double v3 = r * Math.sin(robotAngle) + rightX;
                double v4 = r * Math.cos(robotAngle) - rightX;
                leftFront.setPower(v1 / 1.2);
                rightFront.setPower(v2 / 1.2);
                leftRear.setPower(v3 / 1.2);
                rightRear.setPower(v4 / 1.2);

                arm.controller(new GamepadManager(gamepad2));
            }
        }
    }
}