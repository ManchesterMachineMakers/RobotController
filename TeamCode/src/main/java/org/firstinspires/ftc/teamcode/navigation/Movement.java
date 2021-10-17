package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;

public class Movement {

    public static MyPath calculatePath(Destination startDestination, Destination endDestination, Orientation pose) {

        RobotLog.i("*16221 Navigation::Movement::calculatePath*", "start");
  
        double currentHeading = pose.getPsi() + 90;// +90 points North
        double initialPitch = 25; //PLACEHOLDER
  
        MyPath path = new MyPath();
        path.calcMyPath(startDestination, endDestination, currentHeading);

        RobotLog.i("*16221 Navigation::Movement::calculatePath*", "finished");
        return path;
    }

    public static MyPath rotateForPath(MyPath path, Orientation pose, double angleTolerance, DriveBase driveBase, LinearOpMode opMode) {
        RobotLog.i("*16221 Navigation::Movement::rotateForPath*", "start: tolerance is " + String.valueOf(angleTolerance));

        double errHeading = path.finalHeading - pose.getPsi() + 90; // +90 points North.

        while (Math.abs(errHeading) >= angleTolerance && opMode.opModeIsActive()) {
            //if not aligned straight at target, errHeading=0, then realign
            RobotLog.i("*16221 Navigation::Movement::rotateForPath*", "realigning: error in heading is " + String.valueOf(errHeading));
            angleChange(angleTolerance, errHeading, driveBase, 0.2);
            errHeading = path.finalHeading - pose.getPsi() + 90; // +90 points North.
            opMode.idle();
        }

        RobotLog.i("*16221 Navigation::Movement::rotateForPath*", "aligned within tolerance: error in heading is " + String.valueOf(errHeading));
        driveBase.stop();


        RobotLog.i("*16221 Navigation::Movement::rotateForPath*", "finished");
        return path;
    }

    /**
     * Use this method within a while loop to make adjustments to the robot's angle.
     * @param angleTol
     * @param psi
     * @param driveBase
     * @param speed
     * @return
     */
    public static double angleChange(double angleTol, double psi, DriveBase driveBase, double speed)
    {   
        RobotLog.i("*16221 Navigation::Movement::angleChange*", "start");

        driveBase.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBase.setTravelDirection(DriveBase.TravelDirection.forward);

        if (psi >= -angleTol && psi <= angleTol)
        {
            driveBase.stop();
        }
        else if (psi > angleTol) //rotate counterclockwise
        {
            driveBase.go(new double[]{-speed, speed, -speed, speed});
            //driveBase.go(DriveBase.TravelDirection.pivotLeft, speed);
        }
        else if (psi < -angleTol) //rotate clockwise
        {
            driveBase.go(new double[]{speed, -speed, speed, -speed});
            //driveBase.go(DriveBase.TravelDirection.pivotRight, speed);
        }

        return psi;
    } //END:  AngleChange

    /**
     *
     * @param distance
     * @param speed
     * @param direction
     * @param stage
     * @param driveBase
     * @return
     */
    public static boolean move(double distance, double speed, double direction, int stage, DriveBase driveBase, LinearOpMode opMode) {
        double slip;
        double ticksPerMM = driveBase.motorEncoderEventsPerMM; // 2.3663; // NOTE:  determined this number emperically by running a set # of ticks and dividing by the actual distance traveled
        int targetTicks = (int) Math.round(ticksPerMM * distance);
        int tolerance = 50; // 10 ticks = 4.23 mm traveled (= 0.166 inches)
        String s;

        // set position tolerance
        driveBase.setPositionTolerance(tolerance);
        s = Double.toString(tolerance);
        RobotLog.i("*16221 IsaacArrayDemo*", "tolerance set: " + s);

        // set target position
        DriveBase.TravelDirection travelDirection = (direction == 1 ? DriveBase.TravelDirection.forward : DriveBase.TravelDirection.reverse);
        driveBase.go(travelDirection, speed, targetTicks);
        //driveBase.go(DriveBase.TravelDirection.forward, (direction == 1 ? speed : -speed), targetTicks);
        s = Double.toString(targetTicks);
        RobotLog.i("*16221 IsaacArrayDemo*", "targetTicks set: " + s);
        s = Double.toString(direction);
        RobotLog.i("*16221 IsaacArrayDemo*", "drivebase direction set: " + s);
        s = Double.toString(speed);
        RobotLog.i("*16221 IsaacArrayDemo*", "speed set: " + s);

        while (driveBase.isBusy() && opMode.opModeIsActive()) {
            int[] currentPositions = driveBase.getEncoderPositions();
            RobotLog.i("*16221 IsaacArrayDemo*", "current encoder positions: " + String.valueOf(currentPositions));
            opMode.idle();
        }
        RobotLog.i("*16221 Navigation::Movement::move*", "Completed move.");
        driveBase.stop();


        RobotLog.i("*16221 Navigation::Movement::move*", "finished");
        return true;
    }
    public static boolean isComplete(DriveBase driveBase, int tolerance) {
        boolean complete = false;
        int[] targetTicks = driveBase.getTargetPositions();
        // turn off each motor as it reaches targetTicks + tolerance, to prevent it
        // from being pushed beyond tolerance range by other motors that may still
        // active and thus being turned on again during next pass thru the sending loop
        boolean[] isStopped = driveBase.stopOnTicks(targetTicks, tolerance);
        // is this necessary?  Each motor should stop when it reaches its target, which is
        // essentially what this is doing.
    
        complete = true;
        for (boolean stopped: isStopped) {
          complete = (complete && stopped);
        }
    
        return complete;
      }  // END:  move

}
