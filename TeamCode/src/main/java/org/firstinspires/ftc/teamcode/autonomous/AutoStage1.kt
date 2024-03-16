package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.path.runGrid
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment.*
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.*
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import kotlin.math.PI

open class AutoStage1(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : LinearOpMode() {
    private lateinit var driveBase: DriveBase

    val leftTurn = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> -90.0
        CenterStageAutonomous.Alliance.BLUE -> 90.0
    }

    fun drivePolarAndWait(power: Double, l: Double, theta: Double) {
        driveBase.runPolar(telemetry, power, l, theta)
        telemetry.addData("Motors-Target", driveBase.motors.map { it.targetPosition })
        telemetry.addData("Motors-Actual", driveBase.motors.map { it.currentPosition })
        telemetry.update()
        idle()
        while (opModeIsActive() || driveBase.motors.any {it.isBusy}) {
            telemetry.addData("Motors-Target", driveBase.motors.map { it.targetPosition })
            telemetry.addData("Motors-Actual", driveBase.motors.map { it.currentPosition })
            telemetry.update()
            idle()
        }
    }

    override fun runOpMode() {
        driveBase = DriveBase(this)

        telemetry.isAutoClear = false

        waitForStart()

        // Run to backdrop
        drivePolarAndWait(0.7, 50.00, 3*PI/8)

//        driveBase.yaw(leftTurn, 0.7)
    }
}