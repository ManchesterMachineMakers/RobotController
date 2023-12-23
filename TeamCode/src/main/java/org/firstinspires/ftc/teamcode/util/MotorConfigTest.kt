package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime

@Autonomous(name = "Motor Configuration Test", group = "tests")
class MotorConfigTest : OpMode() {

    val loopTime = ElapsedTime()

    val leftFront = hardwareMap.dcMotor.get("left_front")
    val rightFront = hardwareMap.dcMotor.get("right_front")
    val leftRear = hardwareMap.dcMotor.get("left_rear")
    val rightRear = hardwareMap.dcMotor.get("right_rear")

    override fun init() {
    }

    override fun loop() {
        loopTime.reset()

        while (loopTime.seconds() > 0 && loopTime.seconds() < 2) {
            leftFront.power = 0.2
        }
        while (loopTime.seconds() > 2 && loopTime.seconds() < 4) {
            leftFront.power = 0.0
            rightFront.power = 0.2
        }
        while (loopTime.seconds() > 4 && loopTime.seconds() < 6) {
            rightFront.power = 0.0
            leftRear.power = 0.2
        }
        while (loopTime.seconds() > 6 && loopTime.seconds() < 8) {
            leftRear.power = 0.0
            rightRear.power = 0.2
        }
        while (loopTime.seconds() > 8 && loopTime.seconds() < 10) {
            rightRear.power = 0.0
        }
    }
}