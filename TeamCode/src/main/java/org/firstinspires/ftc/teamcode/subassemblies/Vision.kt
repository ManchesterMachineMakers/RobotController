package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.vision.tfod.TfodProcessor

class Vision(opMode: OpMode) : Subject {
    private val hardwareMap = opMode.hardwareMap
    private val webcam = hardwareMap.get(WebcamName::class.java, "Webcam 1")

    val aprilTag = AprilTagProcessor.Builder().build()
    val tfod = TfodProcessor.Builder()
            // TODO: use custom model
            .build()

    val visionPortal = VisionPortal.Builder()
            .setCamera(webcam)
            .addProcessors(aprilTag, tfod)
            .build()
}