package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.navigation.LineSensor;
import org.firstinspires.ftc.teamcode.util.Names;

import java.util.ArrayList;

/**
 * The color sensor should be mounted on the bot around the middle, front-to-back.
 * It must be a maximum of a couple of inches off the ground.
 */
public class RingSensor extends LineSensor implements Runnable {
    private static ArrayList<LineListener> ringListeners = new ArrayList<>();
    public static void addRingListener(LineListener listener) {
        ringListeners.add(listener);
    }
    public static void clearRingListeners() {
        ringListeners.clear();
    }
    private static void dispatchRingListeners(NormalizedRGBA rgba) {
        for(LineListener listener : ringListeners) {
            listener.handle(rgba);
        }
    }
    public static void startThread(HardwareMap hwMap, LinearOpMode opMode) {
        new Thread(new RingSensor(hwMap, opMode)).start();
    }

    private static NormalizedRGBA referenceColor = new NormalizedRGBA();
    @Override
    public void run() {
        //  this is where we start checking for colors.
        //  when we find one, we will dispatch listeners.

        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            try  {
                // do we see orange?  We will get back either gray or orange.
                NormalizedRGBA orangeColor = detectOrangeColor();
                // if we haven't already seen orange lately and it's orange, report.
                if ((currentRBGA != orangeColor) && (orangeColor != referenceColor)) {
                    RobotLog.i("*** We think we're seeing orange! ***");
                    RobotLog.i("Reference Color is " + String.valueOf(referenceColor.toColor()));
                    RobotLog.i("Current Color is " + String.valueOf(currentRBGA.toColor()));
                    RobotLog.i("Orange Color is " + String.valueOf(orangeColor.toColor()));
                    RobotLog.i("*** Did we see orange? ***");
                    currentRBGA.red = orangeColor.red;
                    currentRBGA.green = orangeColor.green;
                    currentRBGA.blue = orangeColor.blue;
                    currentRBGA.alpha = orangeColor.alpha;
                    dispatchRingListeners(currentRBGA);
                }
                // pause for 1/2 of a second
                opMode.sleep(500);
            } catch (Exception ex) {
                return;
            }
        }
    }

    // initialize this to something that we won't be detecting, so we report our initial color detected.
    // private LinearOpMode opMode;
    //public TouchSensor ringSensor;

    public RingSensor(HardwareMap hwMap, LinearOpMode opMode) {
        // super(hwMap, opMode);
        colorSensor = hwMap.get(NormalizedColorSensor.class, Names.sensor_Rings);
        this.opMode = opMode;
    }

    /**
     * Make this return only orange or gray.
     * |Color|HTML / CSSColor Name|Hex Code#RRGGBB|Decimal Code(R,G,B)|
     * |--- |--- |--- |--- |
     * ||coral|#FF7F50|rgb(255,127,80)|
     * ||tomato|#FF6347|rgb(255,99,71)|
     * ||orangered|#FF4500|rgb(255,69,0)|
     * ||gold|#FFD700|rgb(255,215,0)|
     * ||orange|#FFA500|rgb(255,165,0)|
     * ||darkorange|#FF8C00|rgb(255,140,0)|
     * @return
     * @throws InterruptedException
     */
    public NormalizedRGBA detectOrangeColor() throws InterruptedException {
        // check for color, averaged over 0.2 seconds.
        NormalizedRGBA avgCurrentRBGA = detectAverageColor(0.2);
        // determine percentage of each color to suss out if we're seeing orange.
        // we assume that we start with a GRAY background.
        float sumRGB = avgCurrentRBGA.red + avgCurrentRBGA.green +  avgCurrentRBGA.blue;

        // the idea is that for Orange, red is predominant with green secondary, and blue very small.
        float redPct = avgCurrentRBGA.red/sumRGB;
        float greenPct = avgCurrentRBGA.green/sumRGB;
        float bluePct = avgCurrentRBGA.blue/sumRGB;
        //TODO: make this less of a guess - experiment with more orange things!
        if (redPct > 0.4 && greenPct > 0.2 && bluePct < 0.2) {
            return avgCurrentRBGA;
        }
        return referenceColor;
    }
}
