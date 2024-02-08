package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Blue, Backstage)")
class AutoParkBB : CenterStageAutonomous(Alliance.blue, StartPosition.backstage) {
    override fun runOpMode() {
        initTelemetry()
        runParkOnly()
    }
}