package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
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
        (0f to 0.9f) / Grid ..
        -45.0 / Yaw ..
        Run { driveBase, input ->
            recognitionPosition(detect(vision))
        }

    /**
     * Place the purple pixel (which should be preloaded into the left dropper) next to the duck
     */
    private fun placePurplePixel(arm: Arm) {

        // drop the arm
        arm.drop()
        // place the pixel
        arm.leftRelease.release()
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
                // run to backdrop
                // determine the path based on the duck position, because we already moved the bot
                // rotate to get out of purple pixel placement
                when (duckPosition) {
                    DuckPosition.left -> Noop
                    DuckPosition.center -> 90.0 / Yaw
                    DuckPosition.right -> 180.0 / Yaw
                } ..

                // run to backdrop
                (0f to 1.5f) / Grid ..

                // now that we're at the backdrop, align to the correct apriltag
                // TODO: use the apriltag value to at least try to get to the correct spot
                Run { driveBase, input ->
                    val aprilTags = vision.aprilTag.detections.sortedBy { it.center.x }

                    if (aprilTags.size == 3) {
                        val correctTag = when (duckPosition) {
                            DuckPosition.left -> aprilTags[0]
                            DuckPosition.center -> aprilTags[1]
                            DuckPosition.right -> aprilTags[2]
                        }
                        val placementInfo = arm.getPlacementInfo(1)
                        while (!driveToAprilTag(driveBase, correctTag, placementInfo.distToBase));
                        // put the pixel on the backdrop
                        arm.placePixel(placementInfo)
                        arm.rightRelease.release()
                    } else {
                        log("Incorrect number of AprilTags detected on the backdrop, is the robot drunk?")
                        log("Continuing to next stage")
                    }
                }

    /**
     * Park in the parking area.
     */
    private fun park(duckPosition: DuckPosition) =
        90.0 / Yaw ..
        (0f to when(duckPosition) {
            DuckPosition.left -> 0.75f
            DuckPosition.center -> 1f
            DuckPosition.right -> 1.25f
        }) / Grid ..
        -90.0 / Yaw ..
        (0f to 0.5f) / Grid

    fun runParkOnly() {
        val driveBase = DriveBase(this)
        waitForStart()
        (
                (0f to 0.2f) / Grid ..
                when (alliance) {
                    Alliance.blue -> 90.0
                    Alliance.red -> -90.0
                } / Yaw ..
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

        waitForStart()

        log("detecting a duck")
        val duckPosition = runDetection(vision).run(driveBase, Unit)

        log("detected a duck at ${duckPosition}, delivering purple pixel")

        (
            when(duckPosition) {
                DuckPosition.left -> -135.0
                DuckPosition.center -> -45.0
                DuckPosition.right -> 45.0
            } / Yaw
        ).run(driveBase, Unit)

        placePurplePixel(arm)

        log("delivered the purple pixel, now moving on to yellow")
        placeYellowPixel(arm, vision, duckPosition).run(driveBase, Unit)

        log("parking in the parking area")
        park(duckPosition).run(driveBase, Unit)
    }

    override fun runOpMode() {
        runParkOnly()
    }
}