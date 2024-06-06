package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.vision.tfod.TfodProcessor

class Vision(opMode: LinearOpMode) : Subject {
    private val hardwareMap = opMode.hardwareMap
    private val webcam = hardwareMap.get(WebcamName::class.java, "Webcam 1")

    val dash = DashOpMode.CameraStreamProcessor()

    // http://localhost:63342/RobotController/Vision-9.0.1-javadoc.jar/org/firstinspires/ftc/vision/apriltag/AprilTagProcessor.Builder.html
    val aprilTag = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
            .build()
    //
    val tfod = TfodProcessor.Builder()
        .setModelAssetName("model_20240130_212636.tflite")
            .setModelAspectRatio(16.0/9.0)
            .setModelLabels(listOf("duck"))
            .build()

    val visionPortal = VisionPortal.Builder()
            .setCamera(webcam)
            .addProcessors(aprilTag, tfod, dash)
            .build()

    var ptzControl: PtzControl? = null // let OpMode control
        private set

    init {
        while(visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {}
        ptzControl = visionPortal.getCameraControl(PtzControl::class.java)

        opMode.log("Vision successfully initialized")
    }
}