package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.config

@Autonomous(name = "Motor Configuration Test", group = "tests")
class MotorConfigTest : LinearOpMode() {

    override fun runOpMode() {
        val leftFront = hardwareMap.get(DcMotor::class.java, "left_front")
        val rightFront = hardwareMap.get(DcMotor::class.java, "right_front")
        val leftRear = hardwareMap.get(DcMotor::class.java, "left_rear")
        val rightRear = hardwareMap.get(DcMotor::class.java, "right_rear")

        val motors = listOf(leftFront, rightFront, leftRear, rightRear)

        for (motor in motors) motor.config(DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER)

        waitForStart()

        if (opModeIsActive()) {
            telemetry.addLine("Ports")
            telemetry.addData("left front ", leftFront.portNumber)
            telemetry.addData("right front", rightFront.portNumber)
            telemetry.addData("left rear  ", leftRear.portNumber)
            telemetry.addData("right rear ", rightRear.portNumber)
            telemetry.update()

            while (opModeIsActive()) {
                for (motor in motors) motor.runFor(2000)
                sleep(2000)
            }
        }
    }

    // Run a motor for a set amount of time
    private fun DcMotor.runFor(milliseconds: Long) {
        power = MOTOR_POWER
        sleep(milliseconds)
        power = 0.0
    }

    companion object {
        const val MOTOR_POWER = 0.2
    }
}