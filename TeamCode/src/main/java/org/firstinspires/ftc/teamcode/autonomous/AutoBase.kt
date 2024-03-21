package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.runPolarAndWait
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.squareSize
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.log
import kotlin.math.PI

abstract class AutoBase(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : LinearOpMode() {
    //region Drive Base

    var driveBase: DriveBase? = null

    fun useDriveBase() {
        driveBase = DriveBase(this)
    }

    private fun ensure(sub: Any?, ident: String) {
        if(driveBase == null) log("Attempted to access $ident without calling use$ident()")
    }

    fun polar(l: Double, theta: Double) {
        ensure(driveBase, "DriveBase")
        driveBase?.runPolarAndWait(this::opModeIsActive, telemetry, 0.7, l, theta)
    }

    fun turn(degrees: Double) {
        ensure(driveBase, "DriveBase")
        driveBase?.yaw(degrees, 0.7)
    }

    val left = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> -90.0
        CenterStageAutonomous.Alliance.BLUE -> 90.0
    }

    val right = -left
    //endregion
    //region Arm & Pixel Releases
    var arm: Arm? = null
    var releases: PixelReleases? = null

    fun useArm() {
        arm = Arm(this)
        releases = PixelReleases(this)
    }

    val leftPixel: ReleaseServo?
        get() {
            ensure(releases, "Arm")
            return releases?.left
        }

    val rightPixel: ReleaseServo?
        get() {
            ensure(releases, "Arm")
            return releases?.right
        }

    object on {
        val easel = Arm.WristAlignment.EASEL
        val floor = Arm.WristAlignment.FLOOR
    }

    fun place(pixel: ReleaseServo?, alignment: Arm.WristAlignment) {
        ensure(arm, "Arm")
        arm?.drop(alignment)
        pixel?.open()
    }

    //endregion
    //region Vision
    //endregion

    //region Extensions
    val Int.squares
        get() = this * squareSize/5
    val Double.squares
        get() = this * squareSize/5
    //endregion
}