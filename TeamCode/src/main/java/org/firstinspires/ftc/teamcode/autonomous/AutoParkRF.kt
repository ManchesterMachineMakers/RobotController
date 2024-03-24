package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Red, Front)")
class AutoParkRF : AutoPark(CenterStageAutonomous.Alliance.RED, CenterStageAutonomous.StartPosition.FRONT)