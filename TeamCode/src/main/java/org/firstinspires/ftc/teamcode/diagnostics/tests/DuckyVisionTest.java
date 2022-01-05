package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.sensors.LineSensor;
import org.firstinspires.ftc.teamcode.sensors.RingSensor;
import org.firstinspires.ftc.teamcode.util.Names;
import org.firstinspires.ftc.teamcode.util.RecognitionMatrix;

public class DuckyVisionTest implements Base {
    Telemetry telemetry;
    Camera camera;
    RecognitionMatrix duckyCounter = null;

    public void init(Runner runner, Camera camera) {
        this.camera = camera;
        runner.log("*** Looking for duckies");
        telemetry = runner.opMode.telemetry;
    }

    @Override
    public boolean run(Selectors sel, Runner runner) {
        init(runner, sel.cameraSelector().get());
        Telemetry.Line distanceLine = telemetry.addLine("Looking for duckies: ");
        Telemetry.Item duckyCount = distanceLine.addData("Ducky Count", 0);

        runner.log("Started at " + String.valueOf(runner.opMode.getRuntime()));

        while (runner.opMode.opModeIsActive() && duckyCounter == null) {
            duckyCounter = camera.getDucks();
            duckyCount.setValue(duckyCounter == null ? 0 : "some");
        }

        runner.log("Detected " + String.valueOf(duckyCounter) + " duckies.");
        runner.log("*** ducky-vision test complete at " + String.valueOf(runner.opMode.getRuntime()));
        return false;
    }
}
