package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.sensors.FourCorners;

import java.util.ArrayList;

@Test("Four Corners")
public class FourCornersTest implements Base {
    ArrayList<FourCorners.Distances> measured = new ArrayList<>();
    Telemetry telemetry;

    private void startThreadedDetector(Runner runner, FourCorners.DistanceListener listener) {
        // measure distances
        FourCorners.clearListeners();
        FourCorners.addListener(listener);
        FourCorners.startThread(runner.opMode.hardwareMap, runner.opMode);
    }

    @Override
    public Class<? extends Testable>[] requires() {
        return new Class[0];
    }

    @Override
    public boolean run(Testable[] sel, Runner runner) {
        // Place the robot in a known location on the field, verify coordinates.
        telemetry = runner.opMode.telemetry;
        Telemetry.Line distanceLine = telemetry.addLine("Measured Distances");

        Telemetry.Item xStatus = distanceLine.addData("Detected X", false);
        Telemetry.Item xItem = distanceLine.addData("X Value", 0);

        Telemetry.Item yStatus = distanceLine.addData("Detected Y", false);
        Telemetry.Item yItem = distanceLine.addData("Y Value", 0);

        startThreadedDetector(runner, new FourCorners.DistanceListener() {
            @Override
            public void handle(FourCorners.Distances distances) {
                measured.add(distances);
                xStatus.setValue(distances.statusIRx);
                xItem.setValue(distances.x);
                yStatus.setValue(distances.statusIRy);
                yItem.setValue(distances.y);
                telemetry.update();
            }
        });

        runner.opMode.sleep(3000);
        // now report our findings in the log.
        double sumX = 0, sumY = 0;
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = Double.MIN_VALUE, maxY = Double.MIN_VALUE, avgX, avgY;
        int foundX = 0, foundY = 0;
        for (FourCorners.Distances val :
                measured) {
            if (val.statusIRx) {
                sumX += val.x;
                foundX ++;
                minX = Double.min(val.x, minX);
                maxX = Double.max(val.x, maxX);
            }
            if (val.statusIRy){
                sumY += val.y;
                foundY ++;
                minY = Double.min(val.y, minY);
                maxY = Double.max(val.y, maxY);
            }
        }

        // get the average measured values
        avgX = (sumX / foundX);
        avgY = (sumY / foundY);

        // report average, min, max, inconsistencies in the ability of the X and Y measurements to be obtained
        runner.log("Four Corners Results (X):"
                + " Min X: " + String.valueOf(minX)
                + " Max X: " + String.valueOf(maxX)
                + " Average X: " + String.valueOf(avgX)
                + " Detected X " + String.valueOf(foundX) + " times of " + String.valueOf(measured.size()) + " measurements."
        );
        runner.log("Four Corners Results (Y): "
                + " Min Y: " + String.valueOf(minY)
                + " Max Y: " + String.valueOf(maxY)
                + " Average Y: " + String.valueOf(avgY)
                + " Detected Y " + String.valueOf(foundY) + " times of " + String.valueOf(measured.size()) + " measurements."
        );

        return false;
    }
}
