package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.autonomous.AutoReal
import org.firstinspires.ftc.teamcode.autonomous.CenterStageAutonomous

@Autonomous
class AutoRealBlueBackstage : AutoReal(CenterStageAutonomous.Alliance.BLUE, CenterStageAutonomous.StartPosition.BACKSTAGE)