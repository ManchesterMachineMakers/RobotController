package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Red, Backstage)")
class AutoParkRB : CenterStageAutonomous(Alliance.red, StartPosition.backstage) {
    override fun runOpMode() {
        initTelemetry()
        runParkOnly()
    }
}