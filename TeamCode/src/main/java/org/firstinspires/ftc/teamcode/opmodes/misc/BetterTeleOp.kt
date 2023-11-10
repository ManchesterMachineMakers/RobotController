package org.firstinspires.ftc.teamcode.opmodes.misc

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@TeleOp(name = "Better TeleOp")
class BetterTeleOp: LinearOpMode() {
    override fun runOpMode() {
        // Initializes all the drive motors
        val leftFront: DcMotor = hardwareMap.dcMotor.get("left_front")
        val rightFront: DcMotor = hardwareMap.dcMotor.get("right_front")
        val leftRear: DcMotor = hardwareMap.dcMotor.get("left_rear")
        val rightRear: DcMotor = hardwareMap.dcMotor.get("right_rear")

        telemetry.addLine("Motors initialized")

        // Creates a map of the motors and their associated directions
        val motors = mapOf(
                leftFront to DcMotorSimple.Direction.FORWARD,
                rightFront to DcMotorSimple.Direction.FORWARD,
                leftRear to DcMotorSimple.Direction.FORWARD,
                rightRear to DcMotorSimple.Direction.FORWARD
        )

        // Iterates through all the motors and configures them by their key and value
        for (motor in motors) {
            configDriveMotor(motor.key, motor.value)
        }

        // Telemetry is good
        telemetry.addLine("Motors configured")
        telemetry.addLine("Waiting for start")

        waitForStart() // Pretty self-explanatory

        // Ensures program doesn't try to run even when over
        if(opModeIsActive()) {
            while (opModeIsActive()) {
                // Isaac magic
                val r = hypot(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble())
                val robotAngle =
                    atan2(-gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble()) - Math.PI / 4
                val rightX = gamepad1.right_stick_x
                val v1 = r * cos(robotAngle) + rightX
                val v2 = r * sin(robotAngle) - rightX
                val v3 = r * sin(robotAngle) + rightX
                val v4 = r * cos(robotAngle) - rightX
                leftFront.power = v1 / 1.2
                rightFront.power = v2 / 1.2
                leftRear.power = v3 / 1.2
                rightRear.power = v4 / 1.2
            }
        }
    }

    // Configures a driving motor
    private fun configDriveMotor(motor: DcMotor, direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD) {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER // Runs motor without distance tracking
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE // Brakes motor when stopping
        motor.direction = direction // Sets motors to their specified direction
    }
}