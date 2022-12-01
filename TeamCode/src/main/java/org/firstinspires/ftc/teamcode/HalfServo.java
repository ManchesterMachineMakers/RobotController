package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Initialize Servo to 0.5", group = "Random Bits")
public class HalfServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get("initializeServo");
        waitForStart();
        servo.setPosition(0.5);
        Thread.sleep(1000);
    }
}
