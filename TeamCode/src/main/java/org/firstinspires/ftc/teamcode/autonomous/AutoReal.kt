package org.firstinspires.ftc.teamcode.autonomous

import org.firstinspires.ftc.teamcode.util.log
import kotlin.math.PI

open class AutoReal(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : AutoBase(alliance, startPosition) {
    enum class PixelPlacement {
        LEFT,
        CENTER,
        RIGHT
    }

    override fun runOpMode() {
        //region Initialization
        telemetry.isAutoClear = false
        useDriveBase()
        useArm()
        useVision()
        //endregion

        waitForStart()

        //region Detect duck
        log("Moving forward")
        polar(0.8.squares, 0.0)

        log("Detecting duck")
        val pixelPlacement =
                if(detect() != null)
                    PixelPlacement.CENTER
                else {
                    log("Turning a bit left")
                    turn(PI/4, left)
                    if(detect() != null)
                        PixelPlacement.LEFT
                    else PixelPlacement.RIGHT
                }

        log("Duck placement: ${pixelPlacement.toString()}")
        //endregion

        // Robot is now 45° to the left of the start orientation

        //region Place left pixel on floor
        log("Turning")
        when(pixelPlacement) {
            PixelPlacement.LEFT -> turn(PI/4, left)
            PixelPlacement.CENTER -> Unit
            PixelPlacement.RIGHT -> turn(3*PI/4, right)
        }

        log("Placing left pixel on floor")
        place(leftPixel, on.floor)
        arm?.raise()

        // turn to face toward the backdrop
        log("Turning towards backdrop")
        when(pixelPlacement) {
            PixelPlacement.LEFT -> Unit
            PixelPlacement.CENTER -> turn(PI/2, left)
            PixelPlacement.RIGHT -> turn(PI, left)
        }

        if(pixelPlacement == PixelPlacement.LEFT) {
            log("Going back to start position so we don't run the duck over")
            polar(0.5.squares, PI / 2)
        }
        //endregion

        // Robot is now in the start position, facing toward the backdrop

        //region Run to backdrop
        log("Running to backdrop")
        polar(135.8, if(pixelPlacement == PixelPlacement.LEFT) -PI/8 else 0.0)
        //endregion
        //region Place right pixel on backdrop & park
        log("Placing right pixel on backdrop")
        placeRightOnBackdropAprilTag(pixelPlacement)

        log("Parking")
        polar(1.squares, PI/2)
        //endregion
    }
}