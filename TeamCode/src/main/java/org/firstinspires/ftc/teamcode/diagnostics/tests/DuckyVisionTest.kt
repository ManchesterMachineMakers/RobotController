package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.sensors.Vision
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

fun duckyVisionTest(opMode: DiagnosticsOpMode) = describe<Vision> { vision ->
    val telemetry = opMode.telemetry

    it("initializes successfully") {
        with(vision) {
            initVuforiaForTFOD()
            initTfod()
            activateTfod()
        }
    }

    it("can use TensorFlow") {
        var objectCount = 0
        val distanceLine = telemetry!!.addLine("Looking for muffins: ")
        val duckyCount = distanceLine.addData("Muffin Count", objectCount)
        telemetry.update()
        log("Started at " + opMode.runtime)
        if (opMode.opModeIsActive()) {
            var definiteRecognitions: List<Recognition?>
            while (objectCount < 1 && opMode.opModeIsActive() && !opMode.isStopRequested) {
                vision.spotObjects(telemetry)
                definiteRecognitions = vision.definiteRecognitions
                if (definiteRecognitions.isNotEmpty()) {
                    objectCount = 1
                }
                telemetry.update()
            }
        }

        log("Detected $objectCount muffins.")
    }

    it("can detect VuMarks") {
        if (opMode.opModeIsActive()) {
            vision.deactivateTFOD()
            vision.initVuforiaForVuMarks()
            var trackingResults: VuforiaBase.TrackingResults?
            while (opMode.opModeIsActive() && !opMode.isStopRequested) {
                trackingResults = vision.lookForVuMarks(telemetry)
                if (trackingResults != null && trackingResults.isVisible) {
                    log("Spotted a vuMark " + trackingResults.name)
                    log(trackingResults.toJson())
                }
                opMode.sleep(500)
            }
        }
    }
}