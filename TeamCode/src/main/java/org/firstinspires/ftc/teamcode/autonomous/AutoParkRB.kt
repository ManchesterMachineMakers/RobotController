package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Red, Backstage)")
class AutoParkRB : AutoPark(CenterStageAutonomous.Alliance.RED, CenterStageAutonomous.StartPosition.BACKSTAGE)