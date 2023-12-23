package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision


class CenterStageAutonomous : LinearOpMode() {

    fun detect(vision: Vision): Recognition =
        vision.tfod.recognitions
            .filter { r ->
                r.label == "DUCK" && r.confidence > 0.9
            }
            .sortedBy(Recognition::getConfidence)
            .reversed()
            .firstOrNull() ?: detect(vision)

    fun placePurplePixel(driveBase: DriveBase) {

    }

    override fun runOpMode() {
        val vision = Vision(this)
        val driveBase = DriveBase(this)
    }
}