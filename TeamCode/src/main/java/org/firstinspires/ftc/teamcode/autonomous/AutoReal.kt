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
        polar(1.squares, 0.0)

        log("Turning a bit left")
        turn(45.0, left)

        log("Detecting duck")
        val detection = detect()
        val pixelPlacement =
                if(detection != null)
                    if(detection.centerX < detection.imageWidth/2)
                        PixelPlacement.LEFT
                    else PixelPlacement.CENTER
                else PixelPlacement.RIGHT
        log("Duck placement: ${pixelPlacement.toString()}")
        //endregion

        // Robot is now 45° to the left of the start orientation

        //region Place left pixel on floor
        log("Turning")
        when(pixelPlacement) {
            PixelPlacement.LEFT -> turn(45.0, left)
            PixelPlacement.CENTER -> turn(45.0, right)
            PixelPlacement.RIGHT -> turn(135.0, right)
        }

        log("Placing left pixel on floor")
        place(leftPixel, on.floor)
        arm?.raise()

        // turn to face toward the backdrop
        log("Turning towards backdrop")
        when(pixelPlacement) {
            PixelPlacement.LEFT -> Unit
            PixelPlacement.CENTER -> turn(90.0, left)
            PixelPlacement.RIGHT -> turn(180.0, left)
        }

        log("Going back to start position")
        polar(1.squares, PI/2)
        //endregion

        // Robot is now in the start position, facing toward the backdrop

        //region Run to backdrop
        log("Running to backdrop")
        polar(135.8, -PI/8)
        //endregion
        //region Place right pixel on backdrop & park
        log("Placing right pixel on backdrop")
        place(rightPixel, on.easel)

        log("Parking")
        polar(1.squares, PI/2)
        //endregion
    }
}