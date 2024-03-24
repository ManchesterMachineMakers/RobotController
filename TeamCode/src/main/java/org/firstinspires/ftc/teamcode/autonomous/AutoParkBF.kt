package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Autonomous (Parking Only) (Blue, Front)")
class AutoParkBF : AutoPark(CenterStageAutonomous.Alliance.BLUE, CenterStageAutonomous.StartPosition.FRONT)