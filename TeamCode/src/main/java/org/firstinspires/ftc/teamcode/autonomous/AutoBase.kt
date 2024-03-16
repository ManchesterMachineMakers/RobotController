package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.runPolarAndWait
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.squareSize
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.util.log
import kotlin.math.PI

abstract class AutoBase : LinearOpMode() {
    var driveBase: DriveBase? = null

    fun useDriveBase() {
        driveBase = DriveBase(this)
    }

    fun polar(l: Double, theta: Double) {
        if(driveBase == null) log("Attempted to run polar without calling useDriveBase()")
        driveBase?.runPolarAndWait(this::opModeIsActive, telemetry, 0.7, l, theta)
    }
}