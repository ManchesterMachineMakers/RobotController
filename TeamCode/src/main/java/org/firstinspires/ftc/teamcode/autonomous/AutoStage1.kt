package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.path.runGrid
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment.*
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.*
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI

open class AutoStage1(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : AutoBase() {
    val leftTurn = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> -90.0
        CenterStageAutonomous.Alliance.BLUE -> 90.0
    }

    override fun runOpMode() {
        useDriveBase()

        waitForStart()

        // Run to backdrop
        polar(135.8, 3*PI/8)

//        driveBase.yaw(leftTurn, 0.7)
    }
}