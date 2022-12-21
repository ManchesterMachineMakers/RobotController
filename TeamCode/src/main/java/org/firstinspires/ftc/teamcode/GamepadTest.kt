package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.util.GamepadManager

@TeleOp(name = "Gamepad Test")
class GamepadTest : LinearOpMode() {
    override fun runOpMode() { 
        val gmm = GamepadManager(gamepad1)
        telemetry.addLine("Initialized.")
        telemetry.update()
        RobotLog.i("Initialized gamepad test")
        waitForStart()
        if(opModeIsActive()) {
            RobotLog.i("Starting gamepad test")
            var pressed = telemetry.addData("wasPressed", gmm.wasPressed.joinToString(", ", "[", "]") { it })
            telemetry.update()
            while(opModeIsActive()) {
                gmm.once("x") {
                    RobotLog.i("X pressed")
                }
                gmm.once("y") {
                    RobotLog.i("Y pressed")
                }
                gmm.once("a") {
                    RobotLog.i("A pressed")
                }
                gmm.once("b") {
                    RobotLog.i("B pressed")
                }
                pressed.setValue(gmm.wasPressed.joinToString(", ", "[", "]") { it })
                telemetry.update()
            }
        }
    }
}