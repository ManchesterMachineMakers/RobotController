package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Launch An Airplane")
class LaunchAnAirplane : LinearOpMode() {
    override fun runOpMode() {
        waitForStart();
        hardwareMap.dcMotor.get("airplaneLauncher").power = 0.4
        while(opModeIsActive() && !isStopRequested) {}
    }
}