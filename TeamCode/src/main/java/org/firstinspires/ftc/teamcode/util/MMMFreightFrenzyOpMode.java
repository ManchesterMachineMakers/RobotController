package org.firstinspires.ftc.teamcode.util;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.navigation.Destination;
import org.firstinspires.ftc.teamcode.sensors.FourCorners;
import org.firstinspires.ftc.teamcode.sensors.LineSensor;
import org.firstinspires.ftc.teamcode.navigation.Location;
import org.firstinspires.ftc.teamcode.navigation.MyPath;
import org.firstinspires.ftc.teamcode.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

import java.util.ArrayList;

public abstract class MMMFreightFrenzyOpMode extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime msTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//    private Blinker control_Hub;
//    private Blinker expansion_Hub_2;
//    private HardwareDevice webcam_1;

    // All the different parts
    public DriveBase driveBase;
    public ActiveIntake intake;
    public Delivery delivery;

    // navigation tools

    // driver feedback
//    public Blinkin led;
//    public RobotReport robotReport;
    public int stage  =  0;

    /**
     * Set up all the things that will report back to the driver and into the log.
     */
    public void initReporting() {
        telemetry.setAutoClear(false);
//        led = null; // new Blinkin(hardwareMap);
//        robotReport = new RobotReport(telemetry, null);
    }

    protected void report(String message) {
        telemetry.speak(message);
        telemetry.addLine(message);
        telemetry.update();
    }

    protected void keepTheBeat(int beats) throws InterruptedException {
        while (opModeIsActive() /*&& driveBase.isBusy()*/ && beats > 0) {
            RobotLog.v("Waiting in an English garden");
            Thread.sleep(900); // wait one beat
            RobotLog.v("If the sun doesn't come from sitting in the English rain");
            beats--;
            idle();
        }
    }

    /**
     * Set up all the robot subassemblies.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void initHardware() {
        try {
            driveBase = RobotConfig.CURRENT.getHardware(DriveBase.class, this);
            intake = RobotConfig.CURRENT.getHardware(ActiveIntake.class, this);
            delivery = RobotConfig.CURRENT.getHardware(Delivery.class, this);
        } catch(NoSuchMethodException ignored) {
            ignored.printStackTrace();
        }
        //intake = new ActiveIntake(hardwareMap, this);
        //shooter = new Shooter(hardwareMap);
        //reckoning = new PIDReckoning(hardwareMap, this);(
        //wobbleGoalGrabber = new WobbleGoalGrabber(hardwareMap);

        // vision
//        camera = new Camera(hardwareMap, this);)
    }

    /**
     * Set up all the sensors that will measure for us, providing constant feedback
     * each on its own thread.
     */
    public void initMeasuring() {
        // measure distances
        FourCorners.clearListeners();
        FourCorners.addListener(distances -> {
//                reckoning.updateFourCorners(distances);
        });
        FourCorners.startThread(hardwareMap, this);

        // watch for lines
        LineSensor.clearLineListeners();
        LineSensor.addLineListener(rgba -> {
//                reckoning.updateColor(rgba);
        });
        LineSensor.startThread(hardwareMap, this);

        // watch for rings
        //RingSensor.clearRingListeners();
        //RingSensor.addRingListener(new LineSensor.LineListener() {
        //    @Override
        //    public void handle(NormalizedRGBA rgba) {
        //        // if the color is orange,
        //        // count another ring
        //        RobotReport.ringsOnBot += 1;
        //        if (RobotReport.ringsOnBot >= ActiveIntake.maxRingsAllowedOnBot) {
        //            led.autonomousActionAlert();
        //            intake.stop();
        //        }
        //        robotReport.updateBotStatus();
        //    }
        //});
        //TODO: Start the RingSensor when we have one.
        // RingSensor.startThread(hardwareMap, this);

        // watch for vuMarks
        //VuMarkDetector.clearListeners();
        //VuMarkDetector.addListener(new VuMarkDetector.Listener() {
          //  @Override
          //  public void handle(VuforiaBase.TrackingResults trackingResults) {
          //      reckoning.updateTrackable(trackingResults);
          //  }
        //});
        //TODO: Start the vuMark Detector when we get the right thing in the constructor.

    }

    /**
     * Set up the robot parts to interoperate:
     * reporting systems, all hardware, measurement sensors, visual navigation, and timer.
     */
    public void initOpMode() throws InterruptedException {

        // we set up all our various hardware bits: shooter, grabber, intake, drive base, etc.
        initHardware();
        telemetry.addLine("Hardware Initialized.");
        telemetry.update();

        report("Warning! Please retract the slides completely to zero before running this op mode!");
        keepTheBeat(3);
        report("Slides are now AT ZERO.  If they are not FULLY RETRACTED, you will break them!");
        keepTheBeat(3);
        report("If they are not retracted, stop this OpMode, retract the slides, and restart.");
        keepTheBeat(5);
        // Start it talking
        report("Are you ready?");
        keepTheBeat(1);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        telemetry.addLine("Waiting for Start...");
        telemetry.update();
    }


    /**
     * Drive a course - a list of destinations, one to the next.
     * @param course a list of destinations
     * @return success
     */
    public boolean runCourse(ArrayList<Destination> course) {
        try {
            //TODO: Add telemetry and logging.
            //  for each destination in the course, let's try to go there.
            for (int i = 0; (i < course.size() && opModeIsActive()); i++) {
                if (!runToDestination(course.get(i))) { return false; }
            }
            // when we've finished, return.
            return true;
        }
        catch (Exception ex) {
            return false;
        }
    }

    /**
     * Run to a particular destination.  Get current location, calculate path, rotate to it, drive it.
     * @param destination a destination object
     * @return success
     */
    public boolean runToDestination(Destination destination) {
        //TODO: Add telemetry and logging.
//        telemetry.addLine("running to org.firstinspires.ftc.teamcode.util.pathfinder.Destination " + destination.getName());
//        try {
//            Location currentLocation = reckoning.whereAmI();
//            robotReport.itemFieldPosition.setValue(currentLocation);
//            robotReport.itemFieldOrientation.setValue(currentLocation.orientation);
//            robotReport.itemTravelDestination.setValue(destination);
//
//            MyPath path = Movement.calculatePath(currentLocation, destination, currentLocation.orientation);
//
//            RevBlinkinLedDriver.BlinkinPattern pattern = led.pattern;
//            led.autonomousAction();
//            DcMotorSimple.Direction[] directions = driveBase.getMotorDirections();
//
//            boolean thereYet = reckoning.areWeThereYet(destination, path, reckoning.mmTolerance, reckoning.angleTolerance);
//            if (!thereYet) {
//                // we haven't arrived yet.
//                int encoderTolerance = (int) driveBase.getEncoderValueForRobotMillimeters(reckoning.mmTolerance);
//                // rotate to face our path
//                path = Movement.rotateForPath(path, currentLocation.orientation, reckoning.angleTolerance, driveBase, this);
//
//                // then travel the path
//                boolean driving = Movement.move(path.distance, driveBase.getDriveSpeedPower(DriveBase.DriveSpeed.FAST), path.direction, stage++, driveBase, this);
//
//                reckoning.setCurrentLocation(new org.firstinspires.ftc.teamcode.util.pathfinder.Destination(path.targetName, (float)path.targetX, (float)path.targetY));
//                //while (driving && opModeIsActive()) {
//                //    idle(); // wait for us to finish driving.
//                //    driving = Movement.isComplete(driveBase, encoderTolerance);
//                //}
//            }
//            reckoning.setCurrentLocation(destination);
//            driveBase.setMotorDirections(directions);
//
//            led.blinkinLedDriver.setPattern(pattern);
//
//            robotReport.itemOriginDestination.setValue(destination);
//        } catch (Exception ex) {
//            return false;
//        }
        return true;
    }

    /**
     * Only change facing orientation, not position on the field. Autonomous.
     * @param currentLocation
     * @param destination
     * @return
     */
    public boolean changeFacing(Location currentLocation, Destination destination) {
        //TODO: Add telemetry and logging.

//        try {
//            telemetry.addLine("Change Facing from (" + currentLocation.getX() + "," + currentLocation.getY()
//                    + ") at " + currentLocation.orientation.psi + " heading to " + destination.getName());
//            RevBlinkinLedDriver.BlinkinPattern pattern = led.pattern;
//            led.autonomousAction();
//
//            DcMotorSimple.Direction[] directions = driveBase.getMotorDirections();
//
//            MyPath path = Movement.calculatePath(destination, currentLocation, currentLocation.orientation);
//            Movement.rotateForPath(path, currentLocation.orientation, reckoning.angleTolerance, driveBase, this);
//
//            driveBase.setMotorDirections(directions);
//            led.blinkinLedDriver.setPattern(pattern);
//            return true;
//        } catch (Exception ex) {
//            telemetry.addData("Exception", ex.getStackTrace());
//            return false;
//        }
        return false;
    }

    /**
     * Only change facing orientation, not position on the field. Autonomous
     * @param destination
     * @return
     */
    public boolean changeFacing(Destination destination) {
        //TODO: Add telemetry and logging.
        return false;
//
//        Location currentLocation = reckoning.whereAmI();
//        return changeFacing(currentLocation, destination);
    }

    public boolean changePitch(Orientation currentPose, MyPath path) {
        try {
//            currentPose.theta


        } catch (Exception ex) {

        }
        return false;
    }
}
