package org.firstinspires.ftc.teamcode.subassemblies

import org.manchestermachinemakers.easyop.Subassembly
import org.manchestermachinemakers.easyop.Device
import org.manchestermachinemakers.easyop.Inject
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import com.qualcomm.robotcore.hardware.HardwareMap

class Vision : Subassembly {
    @Device("Webcam 1") lateinit var webcam: WebcamName

    @Inject lateinit var driveBase: DriveBase

    lateinit var aprilTag: AprilTagProcessor
    lateinit var visionPortal: VisionPortal

    override fun customInit(hwMap: HardwareMap) {
        initAprilTag()
    }

    fun initAprilTag() {
        aprilTag = AprilTagProcessor.Builder().build()

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2f)

        visionPortal = VisionPortal.Builder()
            .setCamera(webcam)
            .addProcessor(aprilTag)
            .build()

    }
}