package org.firstinspires.ftc.teamcode.autonomous

import kotlin.math.PI

open class AutoPark(val alliance: CenterStageAutonomous.Alliance, val startPosition: CenterStageAutonomous.StartPosition) : AutoBase(alliance, startPosition) {
    override fun runOpMode() {
        useDriveBase()

        waitForStart()

        polar(15.0, 0.0)
        if(startPosition == CenterStageAutonomous.StartPosition.BACKSTAGE) {
            polar(
                1.5.squares, when (alliance) {
                    CenterStageAutonomous.Alliance.BLUE -> PI / 2
                    CenterStageAutonomous.Alliance.RED -> -PI / 2
                }
            )
        } else {
            polar(
                3.5.squares, when (alliance) {
                    CenterStageAutonomous.Alliance.BLUE -> PI / 2
                    CenterStageAutonomous.Alliance.RED -> -PI / 2
                }
            )
        }
    }
}