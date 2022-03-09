package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.util.RecognitionMatrix;

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

        runner.log("*** Looking for duckies");
        telemetry = runner.opMode.telemetry;
    }

    @Override
    public boolean run(Testable[] sel, Runner runner) throws Exception {
        init(runner, Testable.getOrDie(sel, Vision.class));
        Telemetry.Line distanceLine = telemetry.addLine("Looking for duckies: ");
        Telemetry.Item duckyCount = distanceLine.addData("Ducky Count", 0);

        runner.log("Started at " + runner.opMode.getRuntime());
        if (runner.opMode.opModeIsActive()) {
            while (runner.opMode.opModeIsActive()) {
                camera.spotObjects(telemetry);
            }
        }
//        while (runner.opMode.opModeIsActive() && duckyCounter == null) {
//            duckyCounter = camera.getDucks();
//            duckyCount.setValue(duckyCounter == null ? 0 : "some");
//        }

        runner.log("Detected " + duckyCounter + " duckies.");
        runner.log("*** ducky-vision test complete at " + runner.opMode.getRuntime());
        return false;
    }


}
