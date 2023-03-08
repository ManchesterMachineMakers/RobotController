package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp


class Car(val motor: DcMotor) {
    fun forward() {
        motor.power =  .5
    }

    fun backward() {
        motor.power = -.5
    }
    fun stop() {
        motor.power = 0.0
    }
}

@TeleOp(name = "Motor Tutorial")
class MotorTutorial : LinearOpMode() {
    override fun runOpMode() {
        val motor = hardwareMap.dcMotor.get("left_front")
        val go = Car(motor)
        waitForStart()
        if(opModeIsActive()) {
            go.forward()
            sleep (2000)
            go.backward()
            sleep (2000)
            go.stop()

        }
    }


}

/* 
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

*/