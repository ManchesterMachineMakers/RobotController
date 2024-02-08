package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Red, Front)")
class AutoParkRF : CenterStageAutonomous(Alliance.red, StartPosition.front) {
    override fun runOpMode() {
        initTelemetry()
        runParkOnly()
    }
}