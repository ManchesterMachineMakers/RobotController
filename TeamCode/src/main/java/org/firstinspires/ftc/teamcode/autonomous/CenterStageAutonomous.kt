package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment.*
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.div
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.rangeTo
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm
import org.firstinspires.ftc.teamcode.subassemblies.release
import org.firstinspires.ftc.teamcode.util.bases.BaseArm
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

@Autonomous
open class CenterStageAutonomous(val alliance: Alliance = Alliance.blue, val startPosition: StartPosition = StartPosition.backstage) : LinearOpMode() {

    // Turn the other way if we're on the red alliance!
    public var allianceDirectionCoefficient = if (alliance == Alliance.blue) 1.0 else -1.0
    public var  allianceDirectionDegrees = if (alliance == Alliance.blue) 0.0 else -180.0
    // our backdrop april tag value will be 1-3 or 4-6 depending on alliance.
    public var allianceAprilTagIndexBoost = if (alliance == Alliance.blue) 1 else 3
    private var correctTag: AprilTagDetection? = null

    private var telemetrySetupLine = telemetry.addData("Setup", "Alliance: $alliance, Position: $startPosition")
    private var telemetryActionLine = telemetry.addData("Action", "Initializing")
    private var telemetryPixelLine = telemetry.addData("Pixel", "Both preloaded")
    private var telemetryDuckPosition = telemetry.addData("Duck Position", "Not Yet Detected")
    private var telemetryTagTries = telemetry.addData("April Tag Detection Tries", "Not Yet Detected")
    private var telemetryTagValue = telemetry.addData("April Tag Value", correctTag?.id?:"Not Yet Detected")
    enum class Alliance {
        blue, red
    }

    enum class StartPosition {
        backstage, front
    }

    private enum class DuckPosition {
        left, center, right
    }

    /** Detect a [Recognition] with a [label][Recognition.getLabel] of "duck" and a [confidence][Recognition.getConfidence] greater than 75% (takes the most confident one). THIS FUNCTION WILL CONTINUE TO RUN FOREVER IF NOTHING IS DETECTED. */
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

    /** Get the [DuckPosition] of a TFOD [Recognition] returned by [detect]. */
    private fun recognitionPosition(recognition: Recognition?): DuckPosition {
        if(recognition == null) return DuckPosition.left
        val center = (recognition.left + recognition.right) / 2

        val half = recognition.imageWidth / 2

        // camera view is backwards
        return (
                if (center < half) DuckPosition.center
                else DuckPosition.right
                )
    }

    private fun runDetection(vision: Vision) =
        (0f to 1.2f) / Grid ..
                Run { driveBase, i ->
                    if(detect(vision) != null) DuckPosition.center
                    else ((allianceDirectionDegrees - 45.0)  / Yaw ..
                            Run { _driveBase, input ->
                                if(detect(vision) != null) DuckPosition.right
                                else DuckPosition.left
                            }).run(driveBase, Unit)
                }
    // TODO: Make it look for the center first, then turn to the right, then give up.
    // OR: Make it wait until a motor is not busy before detecting.
    // The routine is detecting the center duck early and then is confused as to which
    // position it is in, as it's in the center of the frame.

    /**
     * Place the purple pixel (which should be preloaded into the left dropper) next to the duck
     */
    private fun placePurplePixel(arm: Arm) {

        // drop the arm
        telemetryActionLine.setValue("Placing on the floor")
        telemetry.update()
        arm.drop()
        // place the pixel
        telemetryActionLine.setValue("Releasing the left pixel")
        telemetry.update()
        arm.leftRelease.release()

        telemetryActionLine.setValue("Raising the arm")
        telemetry.update()
        arm.raise()
    }


    /**
     * Drive to a place in front of the provided apriltag.\
     * @param desiredDistance The desired final distance from the AprilTag in millimetres.
     * @return true if the robot is within tolerance of the desired position
     */
    private fun driveToAprilTag(driveBase: DriveBase, desiredTag: AprilTagDetection, desiredDistance: Double): Boolean {
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
        driveBase.moveRobot(drive, strafe, turn)
        sleep(10)

        return (rangeError < ACCEPTABLE_ERROR && headingError < ACCEPTABLE_ERROR && yawError < ACCEPTABLE_ERROR)
    }

    /**
     * Place the yellow pixel (which should be preloaded into the right dropper) on the correct position on the backdrop.
     * @param duckPosition the position returned by [recognitionPosition]
     */
    private fun placeYellowPixel(arm: Arm, vision: Vision, duckPosition: DuckPosition) =

        // would love to log here??
        // log("running to backdrop")
        // run to backdrop
        (0f to when(startPosition) {
            StartPosition.backstage -> 1.5f
            StartPosition.front -> 4.5f
        }) / Grid ..

        // now that we're at the backdrop, align to the correct apriltag
        // use the apriltag value to at least try to get to the correct spot
        Run { driveBase, input ->
            telemetryActionLine.setValue("looking for AprilTags on the backdrop")
            telemetry.update()

            var tagValue = 0
            for (tries in 1..2) {
                if (this.isStopRequested == false and ((tagValue < 1) or (tagValue > 6))) {
                    log("April Tag Tries: " + tries)
                    // make sure we are detecting fresh AprilTags
                    telemetryActionLine.setValue("Detecting April Tags, try $tries")
                    var aprilTags = vision.aprilTag.detections.sortedBy { it.center.x }
                    if (aprilTags.size > 0) {
                        // if we only found three, assume they are correct
                        if (aprilTags.size == 3) {
                            telemetryTagTries.setValue("found three AprilTags, assume these are good, try $tries")
                            correctTag = when (duckPosition) {
                                DuckPosition.left -> aprilTags[0]
                                DuckPosition.center -> aprilTags[1]
                                DuckPosition.right -> aprilTags[2]
                            }
                            tagValue = correctTag?.id ?: 0
                            telemetryTagValue.setValue(tagValue)
                            telemetry.update()
                        } else {
                            // we found some other number of tags, filter to see if we have what we need
                            log("filtering AprilTags to look for the correct one")
                            var aprilTagWeAreLookingFor = when (duckPosition) {
                                DuckPosition.left -> 1
                                DuckPosition.center -> 2
                                DuckPosition.right -> 3
                            } + allianceAprilTagIndexBoost
                            var correctTags = aprilTags.filter { it.id == aprilTagWeAreLookingFor }
                            if (correctTags.size > 0) {
                                correctTag = correctTags[0]
                            }
                            // if we haven't found the correct tag, just use the first one we've found.
                            log("we don't have the correct tag, we'll just go to the first we found!")
                            if (correctTag == null) correctTag = aprilTags[0]
                            tagValue = correctTag?.id ?: 0
                            telemetryTagValue.setValue(tagValue)
                            if ((tagValue >= 1) and (tagValue <= 6)) {
                                telemetryActionLine.setValue("we found something on the backdrop. going there!")
                                telemetry.update()
                                log("we found something on the backdrop. going there!")
                            } else {
                                telemetryTagTries.setValue("the AprilTag we found was not on the backdrop. That means we are turned around!")
                                telemetry.update()
                                180.0 / Yaw
                                log("turning around and trying again, once.")
                            }
                            telemetry.update()
                        }
                    }
                }
            }
            if (correctTag != null) {
                telemetryActionLine.setValue("placing pixel on pixelRow 1")
                telemetry.update()
                RobotLog.i ("getting placement info for pixelRow 1 on the backdrop")
                val placementInfo = arm.getPlacementInfo(1)
                while (!isStopRequested and !driveToAprilTag(driveBase, correctTag!!, placementInfo.distToBase)) {
                    telemetryActionLine.setValue("Driving to AprilTag")
                    telemetry.update()
                }
                // put the pixel on the backdrop
                telemetryActionLine.setValue("Placing the pixel on the backdrop")
                telemetry.update()
                arm.placePixel(placementInfo)
                arm.rightRelease.release()
            } else {
                RobotLog.i("No AprilTags detected on the backdrop, is the robot drunk?")
                telemetryActionLine.setValue("Dropping a pixel where we are.")
                telemetry.update()
                // drop the pixel where we are, it will likely end up backstage which is 3 points.
                arm.drop()
                arm.rightRelease.release()
                arm.raise()
            }
        }

    /**
     * Park in the parking area.
     */
    @Deprecated("We don't need to park entirely in the area in order to score")
    private fun park(duckPosition: DuckPosition) =
        (90.0 * allianceDirectionCoefficient)/ Yaw ..
        (0f to when(duckPosition) {
            DuckPosition.left -> 0.75f
            DuckPosition.center -> 1f
            DuckPosition.right -> 1.25f
        }) / Grid ..
        (-90.0 * allianceDirectionCoefficient) / Yaw ..
        (0f to 0.5f) / Grid

    fun runParkOnly() {
        val driveBase = DriveBase(this)
        waitForStart()
        (
                (0f to 0.2f) / Grid ..
                        (90.0 * allianceDirectionCoefficient ) / Yaw ..
                (0f to when(startPosition) {
                    StartPosition.backstage -> 2f
                    StartPosition.front -> 5f
                }) / Grid
        ).run(driveBase, Unit)
    }

    fun runFull() {
        val vision = Vision(this)
        val driveBase = DriveBase(this)
        val arm = Arm(this)
        telemetryActionLine.setValue("Initialized")
        telemetry.update()

        waitForStart()

        telemetryActionLine.setValue("detecting a duck")
        telemetry.update()

        val duckPosition = runDetection(vision).run(driveBase, Unit)

        telemetryDuckPosition.setValue(duckPosition)
        telemetryPixelLine.setValue("Purple")
        telemetry.update()

        // Orient to place the purple pixel on the spike mark
        (
            when(duckPosition) {
                DuckPosition.left -> 155.0
                DuckPosition.center -> 0.0
                DuckPosition.right -> 0.0
            } / Yaw
        ).run(driveBase, Unit)

        arm.run {
            armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            armMotor.targetPosition = -3130 // horizontal from stowed
            armMotor.targetPositionTolerance = 20
            armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            armMotor.power = 0.5
            while (armMotor.isBusy) {}
            armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

        placePurplePixel(arm)
        telemetryActionLine.setValue("delivered the purple pixel, now moving on to yellow")
        telemetry.update()


        telemetryPixelLine.setValue("Yellow")
        telemetry.update()
        // Orient toward the backdrop
        (
            ((allianceDirectionCoefficient) * (when(duckPosition) {
                DuckPosition.left -> 0.0
                DuckPosition.center -> 45.0
                DuckPosition.right -> 135.0
            })) / Yaw
        ).run(driveBase, Unit)
        telemetryActionLine.setValue("oriented toward the backdrop, now driving to it")
        telemetry.update()

        // placeYellowPixel drives most of the way to the backdrop,
        // then looks for AprilTags in order to place the pixel correctly.
        // if none are found, it drops the pixel on the floor.
        placeYellowPixel(arm, vision, duckPosition).run(driveBase, Unit)

        telemetryActionLine.setValue("parking in the parking area")
        telemetry.update()
    }

    fun initTelemetry() {
        telemetry.isAutoClear = false
        telemetrySetupLine = telemetry.addData("Setup", "Alliance: $alliance, Position: $startPosition")
        telemetryActionLine = telemetry.addData("Action", "Initializing")
        telemetryPixelLine = telemetry.addData("Pixel", "Both preloaded")
        telemetryDuckPosition = telemetry.addData("Duck Position", "Not Yet Detected")
        telemetryTagTries = telemetry.addData("April Tag Detection Tries", "Not Yet Detected")
        telemetryTagValue = telemetry.addData("April Tag Value", correctTag?.id?:"Not Yet Detected")
        telemetry.update()
    }

    override fun runOpMode() {
        initTelemetry()

        //runParkOnly()
        runFull()
    }
}