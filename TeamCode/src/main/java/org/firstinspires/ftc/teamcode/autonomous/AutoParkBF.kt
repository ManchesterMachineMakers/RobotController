package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Blue, Front)")
class AutoParkBF : CenterStageAutonomous(Alliance.blue, StartPosition.front) {
    override fun runOpMode() {
        initTelemetry()
        runParkOnly()
    }
}