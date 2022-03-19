package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

object VuforiaLocalization {
    fun getRobotPosition(vuforia: VuforiaTrackables, opMode: LinearOpMode): OpenGLMatrix? {
        var currentPosition: OpenGLMatrix? = null
        var targetVisible = false

        while(currentPosition == null && opMode.opModeIsActive()) {
            for(trackable in vuforia) {
                val listener: VuforiaTrackableDefaultListener = trackable.listener as VuforiaTrackableDefaultListener
                if(listener.isVisible) {
                    currentPosition = listener.robotLocation
                    break
                }
            }
        }

        return currentPosition
    }
}
