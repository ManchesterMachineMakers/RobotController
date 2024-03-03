package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Blue, Front)")
class AutoParkBF : CenterStageAutonomous(Alliance.BLUE, StartPosition.FRONT) {
    override fun runOpMode() {
        initTelemetry()
        runParkOnly()
    }
}