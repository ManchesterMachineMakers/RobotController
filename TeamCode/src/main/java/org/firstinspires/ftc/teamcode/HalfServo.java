package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Initialize Servo to 0.5", group = "Random Bits")
public class  HalfServo extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get("initializeServo");
        waitForStart();
        telemetry.addLine("Got here!");
        telemetry.update();
        servo.setPosition(0);
        telemetry.addData("servo position", servo.getPosition());
        telemetry.update();
        sleep(1000);
        servo.setPosition(1);
        telemetry.addData("servo position", servo.getPosition());
        telemetry.update();
        sleep(1000);
        servo.setPosition(0.5);
        telemetry.addData("servo position", servo.getPosition());
        telemetry.update();
        sleep(1000);
    }
}
