package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.sensors.LineSensor;
import org.firstinspires.ftc.teamcode.sensors.RingSensor;

public class RingSensorTest implements Base {
    Telemetry telemetry;
    int ringCounter = 0;

    private void startThreadedDetector(Runner runner, LineSensor.LineListener listener) {
        // measure distances
        RingSensor.clearRingListeners();
        RingSensor.addRingListener(listener);
        RingSensor.startThread(runner.opMode.hardwareMap, runner.opMode);
    }

    @Override
    public boolean run(Selectors sel, Runner runner) {
        // Place the robot in a known location on the field, verify coordinates.
        telemetry = runner.opMode.telemetry;
        Telemetry.Line distanceLine = telemetry.addLine("Looking for orange rings: ");
        Telemetry.Item ringCount = distanceLine.addData("Ring Count", 0);
        Telemetry.Line colorLine = telemetry.addLine("Color Values: ");
        Telemetry.Item red = colorLine.addData("Red", 0);
        Telemetry.Item green = colorLine.addData("Green", 0);
        Telemetry.Item blue = colorLine.addData("Blue", 0);

        runner.log("Started at " + String.valueOf(runner.opMode.getRuntime()));

        startThreadedDetector(runner, new LineSensor.LineListener() {
            @Override
            public void handle(NormalizedRGBA rgba) {
                ringCounter++;
                ringCount.setValue(ringCounter);

                red.setValue(rgba.red);
                green.setValue(rgba.green);
                blue.setValue(rgba.blue);
                telemetry.update();

                runner.log("Detected orange ring at " + String.valueOf(runner.opMode.getRuntime()));
                runner.log("Color Values: Red " + String.valueOf(rgba.red) + "; Green " + String.valueOf(rgba.green) + "; Blue " + String.valueOf(rgba.blue));
            }
        });

        while (runner.opMode.opModeIsActive()) {
            runner.opMode.idle();
        }

        runner.log("Detected " + String.valueOf(ringCounter) + " rings.");
        runner.log("*** Ring sensor test complete at " + String.valueOf(runner.opMode.getRuntime()));
        return false;
    }
}
