package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables
import org.firstinspires.ftc.teamcode.sensors.Vision
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.RobotConfig
import org.firstinspires.ftc.teamcode.util.Subassembly

class Localization(private val opMode: LinearOpMode) : Subassembly {
    val imu = KtHardware.get<IMUManager>(opMode)
    val distance = KtHardware.get<DistanceSensorManager>(opMode)
    val vision = KtHardware.get<Vision>(opMode)

    private val vfGame = VuforiaCurrentGame()
    private val trackables: VuforiaTrackables

    init {
        vision.initTfod()
        vision.initVuforia()
        trackables = initTrackables()
    }

    private fun initTrackables(): VuforiaTrackables {
        val trackables = vision.vuforia.loadTrackablesFromAsset(RobotConfig.CURRENT.getValue("vuforiaTrackableNames"))
        trackables.activate()
        return trackables
    }

    fun getRobotLocation(): OpenGLMatrix? {
        return VuforiaLocalization.getRobotPosition(trackables, opMode) ?: imu.getRobotLocation()
    }
}