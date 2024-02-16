package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Basic Arm Test")
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        Servo wrist = hardwareMap.servo.get("wrist");
        TouchSensor touch = hardwareMap.touchSensor.get("intake");
        // Wrist servo configuration
        wrist.scaleRange(0.25, 0.78);
    // 53% of 300-degree range
        wrist.setDirection(Servo.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // -3130 for horizontal

        telemetry.addLine("Initialization complete - use left stick y for arm, right stick y for wrist");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) {
            Telemetry.Item armTelem = telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            Telemetry.Item wristTelem = telemetry.addData("Wrist Position", wrist.getPosition());
            Telemetry.Item wristAngle = telemetry.addData("Wrist Angle", ((wrist.getPosition() * (0.53 * 300) - 0.5 * 0.53)));
            Telemetry.Item touchTelem = telemetry.addData("Touch Sensor Pressed?", touch.isPressed());
            while (opModeIsActive()) {
                armMotor.setPower(gamepad1.left_stick_y);
                wrist.setPosition(wrist.getPosition() + (gamepad1.right_stick_y / 300));

                armTelem.setValue(armMotor.getCurrentPosition());
                wristTelem.setValue(wrist.getPosition());
                wristAngle.setValue(((wrist.getPosition() * (0.53 * 300) - 0.5 * 0.53)));
                touchTelem.setValue(touch.isPressed());
                telemetry.update();
            }
        }
    }
}