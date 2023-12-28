package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

@Autonomous(name = "Motor Configuration Test", group = "tests")
class MotorConfigTest : LinearOpMode() {

    override fun runOpMode() {
        val leftFront = hardwareMap.dcMotor.get("left_front")
        val rightFront = hardwareMap.dcMotor.get("right_front")
        val leftRear = hardwareMap.dcMotor.get("left_rear")
        val rightRear = hardwareMap.dcMotor.get("right_rear")

        val motors = listOf(leftFront, rightFront, leftRear, rightRear)

        for (motor in motors) motor.config()

        waitForStart()


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                for (motor in motors) motor.runFor(2000, 0.5)
                sleep(2000)
            }
        }
    }

    // Run a motor for a set amount of time
    private fun DcMotor.runFor(milliseconds: Long, power: Double) {
        this.power = power
        sleep(milliseconds)
        this.power = 0.0
    }
}