package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.path.runGrid
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment.*
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.*
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI

class AutoStage1(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : LinearOpMode() {
    val leftTurn = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> -90.0
        CenterStageAutonomous.Alliance.BLUE -> 90.0
    }

    override fun runOpMode() {
        val driveBase = DriveBase(this)

        // Run to backdrop
        driveBase.runPolar(0.7, 1315.0, PI/8)
        driveBase.yaw(leftTurn, 0.7)
    }
}