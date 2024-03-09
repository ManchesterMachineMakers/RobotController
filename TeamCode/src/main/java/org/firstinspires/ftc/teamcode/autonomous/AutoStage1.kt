package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.path.runGrid
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment.*
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.*
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI

open class AutoStage1(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : LinearOpMode() {
    val leftTurn = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> -90.0
        CenterStageAutonomous.Alliance.BLUE -> 90.0
    }

    override fun runOpMode() {
        val driveBase = DriveBase(this)

        waitForStart()

        // Run to backdrop
        driveBase.runPolar(telemetry, 0.7, 0.00, 3*PI/8)
        telemetry.addData("Motors-Target", driveBase.motors.map { it.targetPosition })
        telemetry.addData("Motors-Actual", driveBase.motors.map { it.currentPosition })
        telemetry.update()
        while(!isStopRequested()) {}
        while (driveBase.motors.any {it.isBusy}) {
            telemetry.addData("Motors-Target", driveBase.motors.map { it.targetPosition })
            telemetry.addData("Motors-Actual", driveBase.motors.map { it.currentPosition })
            telemetry.update()
        }
//        driveBase.yaw(leftTurn, 0.7)
    }
}