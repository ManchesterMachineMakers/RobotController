package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.util.RecognitionMatrix;

import java.util.List;

@Test("Ducky Vision Test")
@Requires(Vision.class)
public class DuckyVisionTest implements Base {
    Telemetry telemetry;
    Vision camera;
    RecognitionMatrix duckyCounter = null;

    public void init(Runner runner, Vision camera) {
        this.camera = camera;
        this.camera.initVuforiaForTFOD();
        this.camera.initTfod();
        this.camera.activateTfod();

        runner.log("*** Looking for muffins");
        telemetry = runner.opMode.telemetry;
    }

    @Override
    public boolean run(Testable[] sel, Runner runner) throws Exception {
        init(runner, Testable.getOrDie(sel, Vision.class));
        int objectCount = 0;
        Telemetry.Line distanceLine = telemetry.addLine("Looking for muffins: ");
        Telemetry.Item duckyCount = distanceLine.addData("Muffin Count", objectCount);
        telemetry.update();

        runner.log("Started at " + runner.opMode.getRuntime());
        if (runner.opMode.opModeIsActive()) {
            List<Recognition> definiteRecognitions;
            while (objectCount < 1 && runner.opMode.opModeIsActive() && !runner.opMode.isStopRequested()) {
                camera.spotObjects(telemetry);
                definiteRecognitions = camera.getDefiniteRecognitions();
                if (!definiteRecognitions.isEmpty()) { objectCount = 1; }
                telemetry.update();
            }
        }
//        while (runner.opMode.opModeIsActive() && duckyCounter == null) {
//            duckyCounter = camera.getDucks();
//            duckyCount.setValue(duckyCounter == null ? 0 : "some");
//        }

        runner.log("Detected " + duckyCounter + " muffins.");
        runner.log("*** ducky-vision test complete at " + runner.opMode.getRuntime());

        runner.log("Now looking for vumarks");

        if (runner.opMode.opModeIsActive()) {
            camera.deactivateTFOD();
            camera.initVuforiaForVuMarks();

            VuforiaBase.TrackingResults trackingResults;
            while (runner.opMode.opModeIsActive() && !runner.opMode.isStopRequested()) {
                trackingResults = camera.lookForVuMarks(telemetry);
                if (trackingResults != null && trackingResults.isVisible) {
                    runner.log("Spotted a vuMark " + trackingResults.name);
                    runner.log(trackingResults.toJson());
                }
                runner.opMode.sleep(500);
            }
        }

        runner.log("*** ducky-vision vuMark test complete at " + runner.opMode.getRuntime());
        return false;
    }


}
