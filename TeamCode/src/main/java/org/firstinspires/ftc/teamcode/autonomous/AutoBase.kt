package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.Range
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.runPolarAndWait
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.squareSize
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.PI

abstract class AutoBase(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : LinearOpMode() {
    private fun ensure(sub: Any?, ident: String) {
        if(sub == null) log("Attempted to access $ident without calling use$ident()")
    }
    //region Drive Base

    var mecDriveBase: MecDriveBase? = null

    fun useDriveBase() {
        mecDriveBase = MecDriveBase(this)
    }

    fun polar(l: Double, theta: Double) {
        ensure(mecDriveBase, "DriveBase")
        mecDriveBase?.runPolarAndWait(this::opModeIsActive, telemetry, 0.7, l, theta)
    }

    fun turn(radians: Double, direction: MecDriveBase.TurnDirection) {
        ensure(mecDriveBase, "DriveBase")
        mecDriveBase?.yaw(radians, direction)
    }

    // allow people to write the autonomous as if we're blue
    val left = when(alliance) {
        CenterStageAutonomous.Alliance.RED -> MecDriveBase.TurnDirection.RIGHT
        CenterStageAutonomous.Alliance.BLUE -> MecDriveBase.TurnDirection.LEFT
    }

    val right = when(left) {
        MecDriveBase.TurnDirection.LEFT -> MecDriveBase.TurnDirection.RIGHT
        MecDriveBase.TurnDirection.RIGHT -> MecDriveBase.TurnDirection.LEFT
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

    //region TFOD

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
    //region AprilTags
    var allianceAprilTagIndexBoost = if (alliance == CenterStageAutonomous.Alliance.BLUE) 1 else 3
    fun detectAprilTags(): Array<AprilTagDetection> {
        ensure(vision, "Vision")
        return vision!!.aprilTag.detections.toTypedArray()
    }

    /**
     * Drive to a place in front of the provided apriltag.\
     * @param desiredDistance The desired final distance from the AprilTag in millimetres.
     * @return true if the robot is within tolerance of the desired position
     */
    private fun driveToAprilTag(desiredTag: AprilTagDetection, desiredDistance: Double): Boolean {
        ensure(mecDriveBase, "DriveBase")
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        val SPEED_GAIN = 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        val STRAFE_GAIN = 0.015 //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        val TURN_GAIN = 0.01  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        val MAX_AUTO_SPEED = 0.7   //  Clip the approach speed to this max value (adjust for your robot)
        val MAX_AUTO_STRAFE = 0.7   //  Clip the approach speed to this max value (adjust for your robot)
        val MAX_AUTO_TURN = 0.3   //  Clip the turn speed to this max value (adjust for your robot)

        val ACCEPTABLE_ERROR = 10    //  Acceptable distance error
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        val rangeError = (desiredTag.ftcPose.range - desiredDistance)
        val headingError = desiredTag.ftcPose.bearing
        val yawError = desiredTag.ftcPose.yaw
        // Use the speed and turn "gains" to calculate how we want the robot to move.
        val drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)
        val turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)
        val strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE)

        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        mecDriveBase?.moveRobot(drive, strafe, turn)
        sleep(10)

        return (rangeError < ACCEPTABLE_ERROR && headingError < ACCEPTABLE_ERROR && yawError < ACCEPTABLE_ERROR)
    }

    fun placeRightOnBackdropAprilTag(pixelPlacement: AutoReal.PixelPlacement) {
        ensure(vision, "Vision")
        ensure(mecDriveBase, "DriveBase")
        ensure(arm, "Arm")
        log("looking for AprilTags on the backdrop")

        var correctTag: AprilTagDetection? = null
        var tagValue = 0
        for (tries in 1..2) {
            if (this.isStopRequested == false and ((tagValue < 1) or (tagValue > 6))) {
                log("April Tag Tries: $tries")
                // make sure we are detecting fresh AprilTags
                log("Detecting April Tags, try $tries")
                val aprilTags = vision!!.aprilTag.detections.sortedBy { it.center.x }
                if (aprilTags.isNotEmpty()) {
                    // if we only found three, assume they are correct
                    if (aprilTags.size == 3) {
                        log("found three AprilTags, assume these are good, try $tries")
                        correctTag = when (pixelPlacement) {
                            AutoReal.PixelPlacement.LEFT -> aprilTags[0]
                            AutoReal.PixelPlacement.CENTER -> aprilTags[1]
                            AutoReal.PixelPlacement.RIGHT -> aprilTags[2]
                        }
                        tagValue = correctTag?.id ?: 0
                        log(tagValue.toString())
                    } else {
                        // we found some other number of tags, filter to see if we have what we need
                        log("filtering AprilTags to look for the correct one")
                        val aprilTagWeAreLookingFor = when (pixelPlacement) {
                            AutoReal.PixelPlacement.LEFT -> 1
                            AutoReal.PixelPlacement.CENTER -> 2
                            AutoReal.PixelPlacement.RIGHT -> 3
                        } + allianceAprilTagIndexBoost
                        val correctTags = aprilTags.filter { it.id == aprilTagWeAreLookingFor }
                        if (correctTags.isNotEmpty()) {
                            correctTag = correctTags[0]
                        }
                        // if we haven't found the correct tag, just use the first one we've found.
                        log("we don't have the correct tag, we'll just go to the first we found!")
                        if (correctTag == null) correctTag = aprilTags[0]
                        tagValue = correctTag?.id ?: 0
                        log(tagValue.toString())
                        if ((tagValue >= 1) and (tagValue <= 6)) {
                            log("we found something on the backdrop. going there!")
                        } else {
                            log("the AprilTag we found was not on the backdrop. That means we are turned around!")
                            log("turning around and trying again, once.")
                            turn(PI, left)
                        }
                        telemetry.update()
                    }
                }
            }
        }
        if (correctTag != null) {
            log("placing pixel on pixelRow 1")
            RobotLog.i ("getting placement info for pixelRow 1 on the backdrop")
            val placementInfo = arm!!.getPlacementInfo(1)
            while (!isStopRequested and !driveToAprilTag(correctTag, placementInfo.distToBase)) {
                log("Driving to AprilTag")
            }
            // put the pixel on the backdrop
            log("Placing the pixel on the backdrop")
            telemetry.update()
            arm!!.placePixel(placementInfo)
            rightPixel!!.open()
        } else {
            RobotLog.i("No AprilTags detected on the backdrop, is the robot drunk?")
            log("Dropping a pixel where we are.")
            telemetry.update()
            // drop the pixel where we are, it will likely end up backstage which is 3 points.
//                arm.drop()
            rightPixel!!.open()
            arm!!.raise()
        }
    }
    //endregion
    //endregion
    //region Extensions
    val Int.squares
        get() = this * squareSize/5
    val Double.squares
        get() = this * squareSize/5
    //endregion
}