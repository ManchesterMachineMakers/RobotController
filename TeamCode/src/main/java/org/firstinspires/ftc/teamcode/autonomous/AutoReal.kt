package org.firstinspires.ftc.teamcode.autonomous

import kotlin.math.PI

open class AutoReal(alliance: CenterStageAutonomous.Alliance, startPosition: CenterStageAutonomous.StartPosition) : AutoBase(alliance, startPosition) {
    enum class PixelPlacement {
        LEFT,
        CENTER,
        RIGHT
    }

    override fun runOpMode() {
        //region Initialization
        useDriveBase()
        useArm()
        useVision()
        //endregion

        waitForStart()

        //region Detect duck
        polar(1.squares, 0.0)

        turn(left/2)

        val detection = detect()
        val pixelPlacement =
                if(detection != null)
                    if(detection.centerX < detection.imageWidth/2)
                        PixelPlacement.LEFT
                    else PixelPlacement.CENTER
                else PixelPlacement.RIGHT
        //endregion

        // Robot is now 45° to the left of the start orientation

        //region Place left pixel on floor
        when(pixelPlacement) {
            PixelPlacement.LEFT -> turn(left/2)
            PixelPlacement.CENTER -> turn(right/2)
            PixelPlacement.RIGHT -> turn(right * 3/2)
        }

        place(leftPixel, on.floor)
        arm?.raise()

        // turn to face toward the backdrop
        when(pixelPlacement) {
            PixelPlacement.LEFT -> Unit
            PixelPlacement.CENTER -> turn(left)
            PixelPlacement.RIGHT -> turn(left*2)
        }

        polar(1.squares, PI/2)
        //endregion

        // Robot is now in the start position, facing toward the backdrop

        //region Run to backdrop
        polar(135.8, -PI/8)
        //endregion
        //region Place right pixel on backdrop & park
        place(rightPixel, on.easel)

        // park
        polar(1.squares, PI/2)
        //endregion
    }
}