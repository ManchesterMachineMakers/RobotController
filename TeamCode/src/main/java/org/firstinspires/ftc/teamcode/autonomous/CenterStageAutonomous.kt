package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Path
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.Segment
import org.firstinspires.ftc.teamcode.autonomous.pathfinder.runPath
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.subassemblies.release
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

    /** Detect a [Recognition] with a [label][Recognition.getLabel] of "Pixel" and a [confidence][Recognition.getConfidence] of 90% or above (takes the most confident one). THIS FUNCTION WILL CONTINUE TO RUN FOREVER IF NOTHING IS DETECTED. */
    // TODO: use the custom element
    private fun detect(vision: Vision): Recognition =
            vision.tfod.recognitions
                    .filter { r ->
                        r.label == "Pixel" && r.confidence > 0.9
                    }
                    .sortedBy(Recognition::getConfidence)
                    .reversed()
                    .firstOrNull()
                    ?: run {
                        sleep(20)
                        detect(vision)
                    }

    /** Get the [DuckPosition] of a TFOD [Recognition] returned by [detect]. */
    private fun recognitionPosition(recognition: Recognition): DuckPosition {
        val center = (recognition.left + recognition.right) / 2

        val third = recognition.imageWidth / 3

        return (
                if (center < third) DuckPosition.left
                else if (third < center && center < 2 * third) DuckPosition.center
                else DuckPosition.right
                )
    }


    /**
     * Place the purple pixel (which should be preloaded into the left dropper) next to the duck.
     * @param duckPosition the position returned by [recognitionPosition]
     */
    private fun placePurplePixel(driveBase: DriveBase, arm: Arm, duckPosition: DuckPosition) {

        driveBase.runPath(Path(
                Segment.Grid(0f, 1f),
                when (duckPosition) {
                    DuckPosition.left -> Segment.Yaw(-90.0)
                    DuckPosition.center -> Segment.Noop()
                    DuckPosition.right -> Segment.Yaw(90.0)
                }
        ))


        // drop the arm
        val dropCorrection = arm.drop()
        // place the pixel
        arm.leftRelease.release()
        arm.raise(dropCorrection)
    }


    /**
     * Drive to a place in front of the provided apriltag.
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
    private fun placeYellowPixel(driveBase: DriveBase, arm: Arm, vision: Vision, duckPosition: DuckPosition) {
        // run to backdrop
        // determine the path based on the duck position, because placePurplePixel already moved the bot
        driveBase.runPath(Path(
                // rotate to get out of purple pixel placement
                when (duckPosition) {
                    DuckPosition.left -> Segment.Noop()
                    DuckPosition.center -> Segment.Yaw(-90.0)
                    DuckPosition.right -> Segment.Yaw(-180.0)
                },

                // run to backdrop
                Segment.Grid(0f, 1.5f)
        ))


        // now that we're at the backdrop, align to the correct apriltag
        // TODO: use the apriltag value to at least try to get to the correct spot
        val aprilTags = vision.aprilTag.detections.sortedBy { it.center.x }

        if (aprilTags.size == 3) {
            val correctTag = when (duckPosition) {
                DuckPosition.left -> aprilTags[0]
                DuckPosition.center -> aprilTags[1]
                DuckPosition.right -> aprilTags[2]
            }
            while (!driveToAprilTag(driveBase, correctTag, 1000.0));
        } else {
            log("Incorrect number of AprilTags detected on the backdrop, is the robot drunk?")
            log("Continuing to next stage")
            return
        }


        // put the pixel on the backdrop
        arm.placePixel(driveBase, arm.getPlacementInfo(1))
    }

    /**
     * Park in the parking area.
     */
    private fun park(driveBase: DriveBase) {
        // TODO: adjust based on position
        driveBase.runPath(Path(
                Segment.Yaw(-90.0),
                Segment.Grid(0f, 0.5f)
        ))
    }

    fun runParkOnly() {
        val driveBase = DriveBase(this)
        waitForStart()
        driveBase.runPath(Path(
                Segment.Grid(0f, 0.2f),
                Segment.Yaw(
                        when (alliance) {
                            Alliance.blue -> 90.0
                            Alliance.red -> -90.0
                        }
                ),
                Segment.Grid(0f, when(startPosition) {
                    StartPosition.backstage -> 2f
                    StartPosition.front -> 5f
                })
        ))
    }

    // TODO: support more start positions
    fun runFull() {
        val vision = Vision(this)
        val driveBase = DriveBase(this)
        val arm = Arm(this)

        waitForStart()

        log("detecting a duck")
        val recognition = detect(vision)

        log("detected a duck at ${recognitionPosition(recognition)}, delivering purple pixel")
        placePurplePixel(driveBase, arm, recognitionPosition(recognition))

        log("delivered the purple pixel, now moving on to yellow")
        placeYellowPixel(driveBase, arm, vision, recognitionPosition(recognition))

        log("parking in the parking area")
        park(driveBase)
    }

    override fun runOpMode() {
        runParkOnly()
    }
}