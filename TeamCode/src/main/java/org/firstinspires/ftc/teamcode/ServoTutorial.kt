package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

class Refrigerator(val servo: Servo) {
    fun openDoor() {
        servo.position = 1.0;
    }

    fun closeDoor() {
        servo.position = 0.0;
    }
}

@TeleOp(name = "Servo Tutorial")
class ServoTutorial : LinearOpMode() {
    override fun runOpMode() {
        val servo = hardwareMap.servo.get("initializeServo")
        val fridge = Refrigerator(servo)
        waitForStart()
        if(opModeIsActive()) {
            fridge.openDoor()
            sleep(1000)
            fridge.closeDoor()
            sleep(1000)
        }
    }
}