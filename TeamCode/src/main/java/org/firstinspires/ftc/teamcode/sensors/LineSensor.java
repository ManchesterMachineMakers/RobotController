package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Names;

import java.util.ArrayList;
import java.util.Arrays;
import java.lang.Runnable;

/**
 * The color sensor should be mounted on the bot around the middle, front-to-back.
 * It must be a maximum of a couple of inches off the ground.
 */
public class LineSensor implements Runnable {
    protected LineSensor() {
    }

    public interface LineListener {
        void handle(NormalizedRGBA rgba);
    }
    private static ArrayList<LineListener> lineListeners = new ArrayList<>();
    public static void addLineListener(LineListener listener) {
        lineListeners.add(listener);
    }
    public static void clearLineListeners() {
        lineListeners.clear();
    }
    private static void dispatchLineListeners(NormalizedRGBA rgba) {
        for(LineListener listener : lineListeners) {
            listener.handle(rgba);
        }
    }
    public static void startThread(HardwareMap hwMap, LinearOpMode opMode) {
        new Thread(new LineSensor(hwMap, opMode)).start();
    }

    public void run() {
        //  this is where we start checking for colors.
        //  when we find one, we will dispatch listeners.

        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            try  {
                int primaryColor = detectPrimaryColor();
                if (primaryColor != currentColor) {
                    currentColor = primaryColor;
                    currentRBGA.red = Color.red(currentColor);
                    currentRBGA.green = Color.green(currentColor);
                    currentRBGA.blue = Color.blue(currentColor);
                    currentRBGA.alpha = Color.alpha(currentColor);
                    dispatchLineListeners(currentRBGA);
                }
                // pause for 1/20th of a second
                Thread.sleep(50);
            } catch (Exception ex) {
                return;
            }
        }
    }

    /**
     * Looks for the average color over the last 1/10th of a second; tries to figure out
     * what the predominant color is - WHITE, BLUE, GREEN, RED, GRAY.  These should cover the
     * possibilities for line colors in the game.
     * If the predominant color changes, it dispatches listeners with the new color.
     * Sleeps the thread for 1/20th of a second on each loop, to make sure the other threads get some time too.
     * @throws InterruptedException
     * @return
     */
    public int detectPrimaryColor() throws InterruptedException {
        // check for color, averaged over 0.1 seconds.
        NormalizedRGBA avgCurrentRBGA = detectAverageColor(0.1);
        // determine percentage of each color to suss out the primary color.
        // we assume that we start with a GRAY background.
        int primaryColor = Color.GRAY;
        float sumRGB = avgCurrentRBGA.red + avgCurrentRBGA.green +  avgCurrentRBGA.blue;
        if (avgCurrentRBGA.red/sumRGB > 0.45) {
            primaryColor = Color.RED;
        } else if (avgCurrentRBGA.green/sumRGB > 0.45) {
            primaryColor = Color.GREEN;
        } else if  (avgCurrentRBGA.blue/sumRGB > 0.45) {
            primaryColor = Color.BLUE;
        } else if (avgCurrentRBGA.alpha > 0.45) {
            primaryColor = Color.WHITE;
        }
        return primaryColor;
    }

    // initialize this to something that we won't be detecting, so we report our initial color detected.
    protected int currentColor = Color.CYAN;
    protected NormalizedRGBA currentRBGA = new NormalizedRGBA();
    protected LinearOpMode opMode;
    public NormalizedColorSensor colorSensor;

    Telemetry.Item rgbaLine;
    Telemetry.Item hsvLine;
    Telemetry.Item redLine;
    Telemetry.Item greenLine;
    Telemetry.Item blueLine;
    Telemetry.Item alphaLine;
    Telemetry.Item hueLine;
    Telemetry.Item satLine;
    Telemetry.Item valueLine;
    Telemetry.Item rawAlphaLine;
    Telemetry.Item rawRedLine;
    Telemetry.Item rawGreenLine;
    Telemetry.Item rawBlueLine;


    public LineSensor(HardwareMap hwMap, LinearOpMode opMode) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, Names.sensor_Color);
        this.opMode = opMode;
    }

    public NormalizedRGBA getNormalizedColors() {
        return colorSensor.getNormalizedColors();
    }

    /**
     * Taken from ColorSensor2.java example code. Gets balanced HSV from the RGB sensor.
     * @param colors
     * @return
     */
    public float[] getBalancedColors(NormalizedRGBA colors) {

        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Balance the colors. The values returned by getColors() are normalized relative to the
        // maximum possible values that the sensor can measure. For example, a sensor might in a
        // particular configuration be able to internally measure color intensity in a range of
        // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
        // so as to return a value it the range [0,1]. However, and this is the point, even so, the
        // values we see here may not get close to 1.0 in, e.g., low light conditions where the
        // sensor measurements don't approach their maximum limit. In such situations, the *relative*
        // intensities of the colors are likely what is most interesting. Here, for example, we boost
        // the signal on the colors while maintaining their relative balance so as to give more
        // vibrant visual feedback on the robot controller visual display.
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        int color = colors.toColor();

        // convert the RGB values to HSV values.
        Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);
        return hsvValues;
    }



    public void outputColorValues(NormalizedRGBA rgba, float[] hsvValues) {
        int color;

        if (rgbaLine == null) {
            opMode.telemetry.addLine("******* CurrentColor *********");
            rgbaLine = opMode.telemetry.addData("RGBA", rgba);
            hsvLine = opMode.telemetry.addData("HSV", hsvValues);
            opMode.telemetry.addLine("RGBA values");
            redLine = opMode.telemetry.addData("Red", rgba.red);
            greenLine = opMode.telemetry.addData("Green", rgba.green);
            blueLine = opMode.telemetry.addData("Blue", rgba.blue);
            alphaLine = opMode.telemetry.addData("Alpha", rgba.alpha);
            opMode.telemetry.addLine("HSV values");
            hueLine = opMode.telemetry.addData("Hue", hsvValues[0]);
            satLine = opMode.telemetry.addData("Saturation", hsvValues[1]);
            valueLine = opMode.telemetry.addData("Value", hsvValues[2]);

            color = rgba.toColor();
            opMode.telemetry.addLine("raw Android color: ");
            rawAlphaLine = opMode.telemetry.addData("a", "%02x", Color.alpha(color));
            rawRedLine = opMode.telemetry.addData("r", "%02x", Color.red(color));
            rawGreenLine = opMode.telemetry.addData("g", "%02x", Color.green(color));
            rawBlueLine = opMode.telemetry.addData("b", "%02x", Color.blue(color));
        } else  {
            //opmode.telemetry.addLine("******* CurrentColor *********")
            rgbaLine.setValue(rgba);
            hsvLine.setValue(hsvValues);
            redLine.setValue(rgba.red);
            greenLine.setValue(rgba.green);
            blueLine.setValue(rgba.blue);
            alphaLine.setValue(rgba.alpha);
            hueLine.setValue(hsvValues[0]);
            satLine.setValue(hsvValues[1]);
            valueLine.setValue(hsvValues[2]);

            color = rgba.toColor();
            rawAlphaLine.setValue("%02x", Color.alpha(color));
            rawRedLine.setValue("%02x", Color.red(color));
            rawGreenLine.setValue("%02x", Color.green(color));
            rawBlueLine.setValue("%02x", Color.blue(color));
        }

//                RobotLog.i("Current Color: R(" + String.valueOf(rgba.red) + ") G(" + String.valueOf(rgba.green) + ") B(" + String.valueOf(rgba.blue) + ") A(" + String.valueOf(rgba.alpha) + ")");
//                RobotLog.i("Current Color: H(" + String.valueOf(hsvValues[0]) + ") S(" + String.valueOf(hsvValues[1]) + ") V(" + String.valueOf(hsvValues[2]) + ")");
//                RobotLog.i("Raw Android Color: R(" + String.valueOf(Color.red(color)) + ") G(" + String.valueOf(Color.green(color)) + ") B(" + String.valueOf(Color.blue(color)) + ") A(" + String.valueOf(Color.alpha(color)) + ")");
        opMode.telemetry.update();

    }

    public void outputSummaryValues(int samples, NormalizedRGBA minRGBA, NormalizedRGBA maxRGBA, NormalizedRGBA avgRGBA,
                                    float[] minHSV, float[] maxHSV, float[] avgHSVs) {

        opMode.telemetry.addLine("******* Average Color *********")
                .addData("Red", avgRGBA.red)
                .addData("Green", avgRGBA.green)
                .addData("Blue", avgRGBA.blue)
                .addData("Alpha", avgRGBA.alpha)
                .addData("Hue", avgHSVs[0])
                .addData("Saturation", avgHSVs[1])
                .addData("Value", avgHSVs[2]);
        RobotLog.i("******** AVERAGE VALUES ************");
        RobotLog.i("Average color over " + String.valueOf(samples) + " samples");
        RobotLog.i("Average Color: R(" + String.valueOf(avgRGBA.red) + ") G(" + String.valueOf(avgRGBA.green) + ") B(" + String.valueOf(avgRGBA.blue) + ") A(" + String.valueOf(avgRGBA.alpha) + ")");
        RobotLog.i("Average Color: H(" + String.valueOf(avgHSVs[0]) + ") S(" + String.valueOf(avgHSVs[1]) + ") V(" + String.valueOf(avgHSVs[2]) + ")");

        opMode.telemetry.addLine("******* MAX Color *********")
                .addData("Red", maxRGBA.red)
                .addData("Green", maxRGBA.green)
                .addData("Blue", maxRGBA.blue)
                .addData("Alpha", maxRGBA.alpha)
                .addData("Hue", maxHSV[0])
                .addData("Saturation", maxHSV[1])
                .addData("Value", maxHSV[2]);
        RobotLog.i("******** MAX VALUES ************");
        RobotLog.i("Max color over " + String.valueOf(samples) + " samples");
        RobotLog.i("Max Color: R(" + String.valueOf(maxRGBA.red) + ") G(" + String.valueOf(maxRGBA.green) + ") B(" + String.valueOf(maxRGBA.blue) + ") A(" + String.valueOf(maxRGBA.alpha) + ")");
        RobotLog.i("Max Color: H(" + String.valueOf(maxHSV[0]) + ") S(" + String.valueOf(maxHSV[1]) + ") V(" + String.valueOf(maxHSV[2]) + ")");

        opMode.telemetry.addLine("******* MIN Color *********")
                .addData("Red", minRGBA.red)
                .addData("Green", minRGBA.green)
                .addData("Blue", minRGBA.blue)
                .addData("Alpha", minRGBA.alpha)
                .addData("Hue", minHSV[0])
                .addData("Saturation", minHSV[1])
                .addData("Value", minHSV[2]);
        RobotLog.i("******** MAX VALUES ************");
        RobotLog.i("Min color over " + String.valueOf(samples) + " samples");
        RobotLog.i("Min Color: R(" + String.valueOf(minRGBA.red) + ") G(" + String.valueOf(minRGBA.green) + ") B(" + String.valueOf(minRGBA.blue) + ") A(" + String.valueOf(minRGBA.alpha) + ")");
        RobotLog.i("Min Color: H(" + String.valueOf(minHSV[0]) + ") S(" + String.valueOf(minHSV[1]) + ") V(" + String.valueOf(minHSV[2]) + ")");
        RobotLog.i("******** DONE Averaging Colors ************");
        opMode.telemetry.update();
    }

    public NormalizedRGBA detectAverageColor(double seconds) {
        NormalizedRGBA avgRGBA = new NormalizedRGBA();

        if (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            double colorDetectStartTime = opMode.getRuntime();

            NormalizedRGBA rgba = colorSensor.getNormalizedColors();

            while (rgba == null && !opMode.isStopRequested() && opMode.opModeIsActive()) {
                rgba = colorSensor.getNormalizedColors();
            }
            float[] hsvValues = new float[3];
            assert rgba != null;
            hsvValues = this.getBalancedColors(rgba);

            // telemetry
            outputColorValues(rgba, hsvValues);

            ArrayList<NormalizedRGBA> saveColors = new ArrayList<NormalizedRGBA>();
            saveColors.add(rgba);
            ArrayList<float[]> saveHSVColors = new ArrayList<>();
            saveHSVColors.add(hsvValues);

            //RobotLog.i("******** Averaging Colors ************");
            while (!opMode.isStopRequested() && opMode.opModeIsActive() && opMode.getRuntime() - colorDetectStartTime < seconds) {
                rgba = colorSensor.getNormalizedColors();
                while (rgba == null && !opMode.isStopRequested() && opMode.opModeIsActive()) {
                    rgba = colorSensor.getNormalizedColors();
                }
                saveColors.add(rgba);

                //Color.colorToHSV(rgba.toColor(), hsvValues);
                hsvValues = this.getBalancedColors(rgba);
                saveHSVColors.add(hsvValues);

                //outputColorValues(rgba, hsvValues);

            }

            //opMode.telemetry.addData("End color detection:", opMode.getRuntime());
            float[] avgHSVs = new float[3];

            int sumColors = 0;
            NormalizedRGBA minRGBA = new NormalizedRGBA();
            minRGBA.alpha = 255;
            minRGBA.red = 255;
            minRGBA.green = 255;
            minRGBA.blue = 255;
            NormalizedRGBA maxRGBA = new NormalizedRGBA();
            maxRGBA.alpha = 0;
            maxRGBA.red = 0;
            maxRGBA.green = 0;
            maxRGBA.blue = 0;

            float[] minHSV = new float[3];
            float[] maxHSV = new float[3];
            Arrays.fill(minHSV, 255);
            Arrays.fill(maxHSV, 0);

            NormalizedRGBA sumRGBA = new NormalizedRGBA();
            float[] sumHSVs = new float[3];
            Arrays.fill(sumHSVs, 0);

            for (NormalizedRGBA c :
                    saveColors) {
                Integer.sum(sumColors, c.toColor());
                sumRGBA.red = Float.sum(sumRGBA.red, c.red);
                sumRGBA.green = Float.sum(sumRGBA.green, c.green);
                sumRGBA.blue = Float.sum(sumRGBA.blue, c.blue);
                sumRGBA.alpha = Float.sum(sumRGBA.alpha, c.alpha);

                minRGBA.red = (c.red < minRGBA.red) ? c.red : minRGBA.red;
                maxRGBA.red = (c.red > maxRGBA.red) ? c.red : maxRGBA.red;

                minRGBA.green = (c.green < minRGBA.green) ? c.green : minRGBA.green;
                maxRGBA.green = (c.green > maxRGBA.green) ? c.green : maxRGBA.green;

                minRGBA.blue = (c.blue < minRGBA.blue) ? c.blue : minRGBA.blue;
                maxRGBA.blue = (c.blue > maxRGBA.blue) ? c.blue : maxRGBA.blue;

                minRGBA.alpha = (c.alpha < minRGBA.alpha) ? c.alpha : minRGBA.alpha;
                maxRGBA.alpha = (c.alpha > maxRGBA.alpha) ? c.alpha : maxRGBA.alpha;

            }
            avgRGBA.red = sumRGBA.red/saveColors.size();
            avgRGBA.green = sumRGBA.green/saveColors.size();
            avgRGBA.blue = sumRGBA.blue/saveColors.size();
            avgRGBA.alpha = sumRGBA.alpha/saveColors.size();

            for (float[] c : saveHSVColors) {
                for (int i = 0; i < 3; i++) {
                    sumHSVs[i] = Float.sum(sumHSVs[i], c[i]);
                    minHSV[i] = (c[i] < minHSV[i]) ? c[i] : minHSV[i];
                    maxHSV[i] = (c[i] > maxHSV[i]) ? c[i] : maxHSV[i];
                }
            }
            for (int i = 0; i < 3; i++) {
                avgHSVs[i] = sumHSVs[i]/saveHSVColors.size();
            }
            outputColorValues(avgRGBA, avgHSVs);
            //outputSummaryValues(saveColors.size(), minRGBA, maxRGBA, avgRGBA, minHSV, maxHSV, avgHSVs);
        }
        return avgRGBA;
    }
}
