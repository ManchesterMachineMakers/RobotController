package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.path.runGrid
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment.*
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.*
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI

open class AutoStage1(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : AutoBase(alliance, startPosition) {
    override fun runOpMode() {
        useDriveBase()
        useArm()

        waitForStart()

        // Run to backdrop
        polar(135.8, 3*PI/8)
        turn(left)

        // place pixel
//        place(leftPixel, on.easel)
//
//        // park
//        polar(1.squares, PI/2)
    }
}