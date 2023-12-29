package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.autonomous.path.GridPath
import org.firstinspires.ftc.teamcode.autonomous.path.runGrid
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.log

@Autonomous
class CenterStageAutonomous : LinearOpMode() {

    private enum class DuckPosition {
        left, center, right
    }

    /** Detect a [Recognition] with a [label][Recognition.getLabel] of "DUCK" and a [confidence][Recognition.getConfidence] of 90% or above (takes the most confident one). THIS FUNCTION WILL CONTINUE TO RUN FOREVER IF NOTHING IS DETECTED. */
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

        val third = recognition.imageWidth/3

        return (
                if(center < third) DuckPosition.left
                else if(third < center && center < 2*third) DuckPosition.center
                else DuckPosition.right
                )
    }


    /**
     * Place the purple pixel (which should be preloaded into the left dropper) next to the duck.
     * @param duckPosition the position returned by [recognitionPosition]
     */
    private fun placePurplePixel(driveBase: DriveBase, arm: Arm, duckPosition: DuckPosition) {
        val path = when(duckPosition) {
            DuckPosition.left -> GridPath(1.0, 0.0, 1.0)
            DuckPosition.center -> GridPath(1.0, 0.0, 0.0)
            DuckPosition.right -> GridPath(1.0, 0.0, -1.0)
        }
        driveBase.runGrid(path)

        // drop the arm
        // place the pixel
    }

    /**
     * Place the yellow pixel (which should be preloaded into the right dropper) on the correct position on the backdrop.
     * @param duckPosition the position returned by [recognitionPosition]
     */
    private fun placeYellowPixel(driveBase: DriveBase, arm: Arm, vision: Vision, duckPosition: DuckPosition) {
        // run to backdrop
        // determine the path based on the duck position, because placePurplePixel already moved the bot
        val path = when(duckPosition) {
            DuckPosition.left -> GridPath(1.5, 0.0, 0.0)
            DuckPosition.center -> GridPath(0.0, 1.5, 1.0)
            DuckPosition.right -> GridPath(-1.5, 0.0, 2.0)
        }
        driveBase.runGrid(path)

        // now that we're at the backdrop, align to the correct apriltag

        // put the pixel on the backdrop
    }

    /**
     * Park in the parking area.
     */
    private fun park(driveBase: DriveBase) {
        driveBase.runGrid(0.0, -1.0, -1.0);
    }

    override fun runOpMode() {
        val vision = Vision(this)
        val driveBase = DriveBase(this)
        val arm = Arm(this)

        log("detecting a duck")
        val recognition = detect(vision)

        log("detected a duck at ${recognitionPosition(recognition)}, delivering purple pixel")
        placePurplePixel(driveBase, arm, recognitionPosition(recognition))

        log("delivered the purple pixel, now moving on to yellow")
        placeYellowPixel(driveBase, arm, vision, recognitionPosition(recognition))

        log("parking in the parking area")
        park(driveBase)
    }
}