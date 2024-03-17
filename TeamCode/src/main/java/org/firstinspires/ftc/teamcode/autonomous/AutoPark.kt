package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.runPolarAndWait
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.squareSize
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI

open class AutoPark(val alliance: CenterStageAutonomous.Alliance, val startPosition: CenterStageAutonomous.StartPosition) : AutoBase() {
    override fun runOpMode() {
        useDriveBase()

        waitForStart()

        polar(10.0, 0.0)
        if(startPosition == CenterStageAutonomous.StartPosition.BACKSTAGE) {
            polar(
                1.5 * squareSize/5, when (alliance) {
                    CenterStageAutonomous.Alliance.BLUE -> PI / 2
                    CenterStageAutonomous.Alliance.RED -> -PI / 2
                }
            )
        } else {
            telemetry.addLine("This autonomous does not support front start positions yet.")
            telemetry.update()
        }
    }
}