package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Blue, Backstage)")
class AutoParkBB : AutoPark(CenterStageAutonomous.Alliance.BLUE, CenterStageAutonomous.StartPosition.BACKSTAGE)