package org.firstinspires.ftc.teamcode.navigation;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.sensors.FourCorners;
import org.firstinspires.ftc.teamcode.sensors.IMUSensor;

import java.util.ArrayList;
import java.util.Hashtable;

public class PIDReckoning {

    LinearOpMode opMode;
    public static BNO055IMU imu;
    public static Orientation pose;
    Camera camera;

    Hashtable<String, Double> vuMarkTiming;
    Hashtable<String, VuforiaBase.TrackingResults> vuMarkResults;
    ArrayList<FourCorners.Distances> fourCorners;
    ArrayList<Double> fourCornersTiming;
    public final double timeTolerance = 200; // last 2/10ths of a second should be reasonable
    public final float mmTolerance = 100; //TODO: This is just a guess of 1cm. No idea what it should be.
    public final float angleTolerance = 2; //TODO:  This is just a  guess of 2 degrees?
    // may want timing on the last distance sensor update too?
    // it should run ever 50 ms or so.
    NormalizedRGBA color;

    float psi = 0; // z angle
    float theta = 0; // y angle
    float phi = 0; // x angle
    org.firstinspires.ftc.robotcore.external.navigation.Orientation angles;
    float x = 0;
    float y = 0;
    float z = 0; // z should always be 0 unless we're flying, whee!

    //declare variables for PID control system (proportional-integral-derivative)
    double errX = 0;
    double errY = 0;
    double errZ = 0;
    double errX_prev = 0;
    double errY_prev = 0;
    double errZ_prev = 0;
    double dvErrX = 0;
    double dvErrY = 0;
    double dvErrZ = 0;
    double iErrX = 0;
    double iErrY = 0;
    double iErrZ = 0;
    double IRerrX = 0;   // used for IR distance sensor, X direction
    double IRerrY = 0;   // used for IR distance sensor, Y direction
    //********************PID COEFFICIENTS***********************
    double Kp = 0.000833;
    double Kd = -0.0006244;
    double Ki = -0.0000888;
    //***********************************************************

    public int step = 0;

    /**
     * A class to help bring together all the different sensor inputs.
     * It will handle the IMU,
     * but you need to pass in values from:
     *  the Line Sensor 
     *  the Distance Sensors
     *  the VuMarks
     */
    public PIDReckoning(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        //if (imu == null) {
            imu = new IMUSensor(hardwareMap).initIMU();
            pose = new Orientation(imu);
        //}
        //corners = new FourCorners(hardwareMap, opMode);
        //camera = new Camera(hardwareMap, opMode);

        // initialize a place for each tracking result. Do we need to?
        vuMarkResults = new Hashtable<>();
        vuMarkTiming = new Hashtable<>();
//        for (String trackable :
//                VuforiaCurrentGame.TRACKABLE_NAMES) {
//            vuMarkResults.put(trackable, );
//            vuMarkTiming.put(trackable, (double) 0);
//        }
    }

    /**
     * Returns a org.firstinspires.ftc.teamcode.util.pathfinder.Destination representing the bot's current position on the field,
     * based on the IMU, the distance sensors, and the vuMarks
     */
    public Location whereAmI() {
        // use the Orientation class to do our math.  Our pose instance has our IMU in it, so it will get current positions every time.
        pose.update();
        psi = (float) pose.psi;
        theta = (float) pose.theta;
        phi = (float) pose.phi;

        angles = pose.angles;
        //ArrayList<Axis> foo = new ArrayList<Axis>(((OrientationSensor) imu).getAngularOrientationAxes());

        Acceleration gravity = pose.gravity; // imu.getGravity();

        if (psi >= 80  && psi <=100) {
            // positioning from the four distance sensors
            float[] coordinates = avgFourCornersPosition();

            if (coordinates[0] > 0) {
                // we have an x value from the distance sensors
                x = coordinates[1];
            }
            if (coordinates[2] > 0) {
                // we have a y value from the distance sensors
                y = coordinates[3];
            }
        }
        // positioning from the vuMarks
        //double[] vuMarkPosition = avgVuMarkPosition();

        //if (vuMarkPosition[0] > 0) {
        //  if (coordinates[0] > 0) {
        //    // we have both - average the two values, as we have no idea which to trust more.
        //  x = (coordinates[1] + (float)(vuMarkPosition[1])) / 2;
        //    } else {
        //      // we have an x value from the vuMarks
        //    x = (float)vuMarkPosition[1];
        //}
        //}

        //if (vuMarkPosition[0] > 0) {
          //  if (coordinates[2] > 0) {
            //    // we have both - average the two values, as we have no idea which to trust more.
              //  y = (coordinates[3] + (float)(vuMarkPosition[2])) / 2;
        //    } else {
                // we have a y value from the vuMarks
          //      y = (float)vuMarkPosition[2];
            //}
        //}

        // we could try double-checking the IMU with the VuMarks, but we probably don't need to?

        Location location = new Location(imu,
                "Current Location",
                x, y, phi, theta, psi
        );
        location.orientation = pose;
        return location;
    }

    /**
     * Determines whether the bot has reached a predefined org.firstinspires.ftc.teamcode.util.pathfinder.Destination.
     * Checks the difference between measured values and current values.
     * If it's supposed to be on a line, it will also check the LineSensor color.
     * If within the given tolerances, it will return the predefined org.firstinspires.ftc.teamcode.util.pathfinder.Destination.
     * If not, it will return a org.firstinspires.ftc.teamcode.util.pathfinder.Destination named "difference".
     */
    public boolean areWeThereYet(Destination destination, MyPath path, float mmTolerance, float angleTolerance) {

        //TODO: This may not be precise enough, because the color sensor is on the back of the robot.
        int lineColor = 0;
        if ((destination == FieldDestinations.SLC) ||
            (destination == FieldDestinations.SRC) ||
            (destination == FieldDestinations.SLT) ||
            (destination == FieldDestinations.SRT)
        ) {
            lineColor = Color.RED;
        }
        if ((destination == FieldDestinations.LLP)
        ) {
            lineColor = Color.WHITE;
        }

        if (
            Math.abs(path.deltaX) <= mmTolerance &&
            Math.abs(path.deltaY) <= mmTolerance &&
            Math.abs(path.leastRot) <= angleTolerance &&
            (lineColor == 0 || lineColor == color.toColor()))
        {
            // yes, we're there, or close enough.  Return the named destination
            return true;
        } else {
            // nope, not yet.
            return false;
        }
    }

    /**
     * Pass in the latest Vuforia VuMark tracking.
     */
    public void updateTrackable(VuforiaBase.TrackingResults trackingResults) {
        vuMarkResults.put(trackingResults.name, trackingResults);
        vuMarkTiming.put(trackingResults.name, opMode.getRuntime());
    }

    /**
     * Returns the average position from the distance sensors.
     * Average the values to smooth out noise
     */
    private float[] avgFourCornersPosition() {
        float sumX = 0;
        float sumY = 0;
        float countX = 0;
        float countY = 0;
        float[] coordinates = new float[4];
        // will be 0 if not filled by a value, so we include the count of values as well.
        // format: { countX, x, countY, y }
        double runtime = opMode.getRuntime();
        if (fourCorners != null && fourCorners.size() > 0 && fourCornersTiming.size() == fourCorners.size()) {
            for (int i = fourCorners.size(); i > 0 && fourCornersTiming.get(i) >= runtime - timeTolerance; i--) {
                if (fourCorners.get(i).statusIRx) {
                    sumX += (float)fourCorners.get(i).x;
                    countX += 1;
                }
                if (fourCorners.get(i).statusIRy) {
                    sumY += (float)fourCorners.get(i).y;
                    countY += 1;
                }
            }
            coordinates[0] = countX;
            coordinates[1] = sumX / countX;
            coordinates[2] = countY;
            coordinates[3] = sumY / countY;
        }
        return coordinates;
    }
    
    /**
     * Returns the average position from the vuMark results.
     * Average the values to smooth out noise
     */
    private double[] avgVuMarkPosition() {
        float sumX = 0;
        float sumY = 0;
        float sumZ = 0;
        float sumAngleX = 0; // roll
        float sumAngleY = 0; // pitch
        float sumAngleZ = 0; // yaw
        float count = 0;

        // take a running average of the vuMark tracked positions together.
        double runtime = opMode.getRuntime();
        for (String key :
                VuforiaCurrentGame.TRACKABLE_NAMES) {
            if (vuMarkTiming.getOrDefault(key, (double) 0) >= runtime - timeTolerance) {
                // recent enough to count
                VuforiaBase.TrackingResults tracked = vuMarkResults.get(key);
                if (tracked != null) {
                    count += 1;
                    sumX += tracked.x;
                    sumY += tracked.y;
                    sumZ += tracked.z;
                    sumAngleX += tracked.xAngle;
                    sumAngleY += tracked.yAngle;
                    sumAngleZ += tracked.zAngle;
                }
            }
        }
        double[] fused = new double[7];
        fused[0] = count;
        fused[1] = (sumX / count);
        fused[2] = (sumY / count);
        fused[3] = (sumZ / count);
        fused[4] = (sumAngleX / count);
        fused[5] = (sumAngleY / count);
        fused[6] = (sumAngleZ / count);

        return fused;
    }

    /**
     * Pass in the latest distance sensor values.
     */
    public void updateFourCorners(FourCorners.Distances position) {
        fourCorners.add(position);
        fourCornersTiming.add(opMode.getRuntime());
    }

    /**
     * Pass in the latest line sensor values.
     */
    public void updateColor(NormalizedRGBA rgba) {
        color = rgba;
    }

    /**
     * Pass in the destination we just arrived at.
     * @param destination
     */
    public void setCurrentLocation(Destination destination) {
        x = destination.getX();
        y = destination.getY();
    }

}
