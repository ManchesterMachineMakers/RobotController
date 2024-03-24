package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.runPolarAndWait
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.squareSize
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Vision
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

    fun turn(degrees: Double, direction: DriveBase.TurnDirection) {
        ensure(driveBase, "DriveBase")
        driveBase?.yaw(degrees, direction)
    }

    // allow people to write the autonomous as if we're blue
    val left = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> DriveBase.TurnDirection.RIGHT
        CenterStageAutonomous.Alliance.BLUE -> DriveBase.TurnDirection.LEFT
    }

    val right = when(left) {
        DriveBase.TurnDirection.LEFT -> DriveBase.TurnDirection.RIGHT
        DriveBase.TurnDirection.RIGHT -> DriveBase.TurnDirection.LEFT
    }
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
    var vision: Vision? = null

    fun useVision() {
        vision = Vision(this)
    }

    /** Detect a [Recognition] with a [label][Recognition.getLabel] of "duck" and a [confidence][Recognition.getConfidence] greater than 75% (takes the most confident one). THIS FUNCTION WILL CONTINUE TO RUN FOREVER IF NOTHING IS DETECTED. */
    fun detect(): Recognition? {
        ensure(vision, "Vision")
        return detect(vision!!)
    }
    private fun detect(vision: Vision) = detect(vision, 0)
    private fun detect(vision: Vision, iteration: Int): Recognition? =
            vision.tfod.recognitions
                    .filter { r ->
                        r.label == "duck" && r.confidence > 0.75
                    }
                    .sortedBy(Recognition::getConfidence)
                    .reversed()
                    .firstOrNull()
                    ?: if (opModeIsActive() && !isStopRequested && iteration < 100 /* approx. 2 seconds */) { run {
                        sleep(20)
                        detect(vision, iteration + 1)
                    } } else { null }

    val Recognition.centerX: Float
        get() = (left + right) / 2
    //endregion

    //region Extensions
    val Int.squares
        get() = this * squareSize/5
    val Double.squares
        get() = this * squareSize/5
    //endregion
}