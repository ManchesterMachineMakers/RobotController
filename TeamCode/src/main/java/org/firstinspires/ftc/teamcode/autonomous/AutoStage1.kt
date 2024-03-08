package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment.*
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.*
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase

class AutoStage1(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : LinearOpMode() {
    val leftTurn = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> -90
        CenterStageAutonomous.Alliance.BLUE -> 90
    }

//    val path =
//            (0f to 1f) / Grid ..

    override fun runOpMode() {
        val driveBase = DriveBase(this)


    }
}