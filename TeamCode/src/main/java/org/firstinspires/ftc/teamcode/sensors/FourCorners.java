package org.firstinspires.ftc.teamcode.sensors;//***************************************************************************************************************************
//
//  This class determines the bot's (X,Y) field position using two to four REV 2M Distance sensors.
//
//  It is useful in field corners, since the REV 2M Distance Sensor's maximum range is 1.2m
//  (ie, about 4 feet, or 2 field floor tiles).
//
//  It is also useful when VuMarks are not visible and/or when the bot is so far away from the visible VuMark 
//  that the position returned by VuForia for the VuMark is not sufficiently accurate.
//
//  On an 8' x 12' FTC field, this class will accurately find the bot's location anywhere within 4' (X & Y) 
//  of the field's 4 corners.  This is adequate for accurate Y axis position (East-West) on an 8' wide field.
//  For X axis position (North-South), it will accurately determine position as long as the bot is within 
//  4' of the North or South wall.  It will not return an accurate value for X axis position 
//  in the middle 4' of the 12' axis. 
//
//  On a traditional FTC 11'9"' x 11'9" field, the class can only determine X-Y position accurately within the
//  4' x 4' squares of each field's corner. Due to sensor limits, it cannot return accurate position values 
//  in the middle of the field.
//
//  NOTE:  The REV 2m Distance Sensor uses the ST Micro VL53LOX device, which has an accuracy rating of 
//  +/- 4% of the measured value.    At 304.8 mm measured distance (1 foot), this is +/- 12.2mm (about +/- 0.5").
//  It should not be used to determine initial placement position of the bot on the field, since at field center on a 8' wide
//  field, its accuracy limits could cause the starting position to be off by +/- 48.8mm (about +/- 2").
//
//  NOTE:  The class assumes the bot has been rotated to face North (90 degrees) prior to sampling the sensors.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Names;

import java.util.ArrayList;

/**
 * Took Mike's example distance sensor detection and followed Aleks's Runnable pattern
 * for a threaded sensor that periodically reports.
 */
public class FourCorners implements Runnable {

    /**
     * Must be implemented by a listener class and added into the listeners array with
     * addListener(myDistanceListenerClass);
     */
    public interface DistanceListener {
        void handle(Distances distances);
    }
    private static ArrayList<FourCorners.DistanceListener> listeners = new ArrayList<>();

    /**
     * Add listeners who will receive distance sensor reports from the FourCorners.
     * Listeners must implement the DistanceListener interface, which is just the "handle" method
     * that takes the double[] array that will be reported from the getPosition() calls.
     * @param listener
     */
    public static void addListener(FourCorners.DistanceListener listener) {
        listeners.add(listener);
    }

    /**
     * Clear out the listeners.  This should be done before any listeners are added
     * when initializing the OpMode, or subsequent runs of the OpMode will run into
     * reporting problems.
     */
    public static void clearListeners() {
        listeners.clear();
    }

    /**
     * Called internally to report to all added listeners the result of the distance sensor calculations.
     * @param detected
     */
    private static void dispatchListeners(Distances distances) {
        for(DistanceListener listener : listeners) {
            listener.handle(distances);
        }
    }

    /**
     * Call this method from the OpMode to activate the distance sensor.
     * @param hwMap
     * @param opMode
     */
    public static void startThread(HardwareMap hwMap, LinearOpMode opMode) {
        new Thread(new FourCorners(hwMap, opMode)).start();
    }

    /**
     * detect and report distances twice every second.  Sleep in between.
     */
    @Override
    public void run() {
        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            try  {
                dispatchListeners(getPosition());
                // pause for 1/2 second
                Thread.sleep(500);
            } catch (Exception ex) {
                return;
            }
        }
    }

    public class Distances {
        public boolean statusIRx; // composite status of X direction sensors
        public boolean statusIRy; // composite status of Y direction sensors
        public double x;
        public double y;
    }

    //declare all variables
    private LinearOpMode opMode;

    public DistanceSensor rightIR;
    public DistanceSensor leftIR;
    public DistanceSensor frontIR;
    public DistanceSensor rearIR;

    public String Name;
    private double[] actual;
    double distRight; // raw distance measured by sensor, in mm
    double distLeft;  // raw distance measured by sensor, in mm
    double distFront; // raw distance measured by sensor, in mm
    double distRear;  // raw distance measured by sensor, in mm
    double offsetLeft = 154.3; //offset in mm from center of bot, in viewing direction of sensor
    double offsetRight = 159.4; //offset in mm from center of bot, in viewing direction of sensor
    double offsetFront = 161.8; //offset in mm from center of bot, in viewing direction of sensor
    double offsetRear = 209.55; //offset in mm from center of bot, in viewing direction of sensor
    boolean statusLeft;  // sensor status, must be within maxIRrange to be true
    boolean statusRight; // sensor status, must be within maxIRrange to be true
    boolean statusFront; // sensor status, must be within maxIRrange to be true
    boolean statusRear;  // sensor status, must be within maxIRrange to be true
    boolean statusIRx; // composite status of X direction sensors
    boolean statusIRy; // composite status of Y direction sensors
    double IRx;  // position on field, X coordinate
    double IRx_sum; // for rolling average
    double IRy;  // position on field, Y coordinate
    double IRy_sum; // used for rolling average
    double countX; //count of valid X measurements during rolling average window
    double countY; //count of valid Y measurements during rolling average window
    double maxIRrange = 1200; // max range in mm of REV 2M Distance Sensor (uses ST Micro VL53L0X)

    //constructor
    public FourCorners(HardwareMap hardwareMap, LinearOpMode opMode) {
        //map hardware and declare variables for IR distance sensors
        this.opMode = opMode;

        rightIR = hardwareMap.get(DistanceSensor.class, Names.range_Right);
        leftIR = hardwareMap.get(DistanceSensor.class, Names.range_Left);
        frontIR = hardwareMap.get(DistanceSensor.class, Names.range_Front);
        rearIR = hardwareMap.get(DistanceSensor.class, Names.range_Rear);

        actual = new double[2];

//      Name = name;
    } // END:  FourCorners constructor
    
    //methods
    void update(){
        
        IRx_sum = 0;
        IRy_sum = 0;
        countY = 0;
        countX = 0;

      for(int i = 0; i < 5; i++){ // NOTE:  # loops = # samples in rolling average.  Loop time = # samples * 33 ms/sample
        //reset status of all IR sensors to false on each pass
        statusRight = false;
        statusLeft = false;
        statusFront = false;
        statusRear = false;
        statusIRx = false;
        statusIRy = false;
        
        //get distance from each IR sensor (bot at 90 degrees), and set status to true on each measuing within range
        distRight = rightIR.getDistance(DistanceUnit.MM);
        if(distRight <= maxIRrange){
                statusRight = true;
            }         
        distLeft = leftIR.getDistance(DistanceUnit.MM);
        if(distLeft <= maxIRrange){
                statusLeft = true;
            }     
        distFront = frontIR.getDistance(DistanceUnit.MM);
        if(distFront <= maxIRrange){
                statusFront = true;
            }    
        distRear = rearIR.getDistance(DistanceUnit.MM);
        if(distRear <= maxIRrange){
                statusRear = true;
            }             

        // calculate IRx and IRy based on status of sensors on opposite sides
        if(statusRight==true && statusLeft==false){
            IRy = -(1790 - distRight - offsetRight);
            statusIRy = true;
        } else if (statusRight==false && statusLeft==true){
            IRy = 598 - distLeft - offsetLeft;
            statusIRy = true;
        } else if (statusRight==true && statusLeft==true){
            IRy = ((598 - distLeft - offsetLeft)+(-(1790-distRight-offsetRight)))/2;
            statusIRy = true;
        } else {
            IRy = 0;
            statusIRy = false;
        }
        
        if(statusIRy){
            IRy_sum = IRy_sum + IRy;
            countY += 1; 
        } // END: if(statusIRy)
        
        if(statusFront==true && statusRear==false){
            IRx = (1790 - distFront - offsetFront);
            statusIRx = true;
        } else if (statusFront==false && statusRear==true){
            IRx = -(1790-distRear-offsetRear);
            statusIRx = true;
        } else if (statusFront==true && statusRear==true){
            IRx = ((1790 - distFront - offsetFront)+(-(1790-distRear-offsetRear)))/2;
            statusIRx = true;
        } else {
            IRx = 0;
            statusIRx = false;
        }

        if(statusIRx){
            IRx_sum = IRx_sum + IRx;
            countX +=1;
        } // END:  if(statusIRx)
      }  // END:  for loop
      
      IRy = IRy_sum / countY;  // rolling average, to reduce noise in Y value
      IRx = IRx_sum / countX;  // rolling average, to reduce noise in X value

    }  //END:  update method

    /**
     * Returns the position on the field as detected by the four distance sensors.
     * If all are out of range, (x,y) values will be (0,0).
     * @return
     */
    public Distances getPosition(){
        update();
        Distances dist = new Distances();
        dist.x = this.IRx;
        dist.y = this.IRy;
        dist.statusIRx = this.statusIRx;
        dist.statusIRy = this.statusIRy;

        return dist;

  /*----for debugguing use only, to set distance offsets for each sensor -----
        telemetry.addData("statusLeft: ",statusLeft);
        telemetry.addData("statusRight: ",statusRight);
        telemetry.addData("statusFront: ",statusFront);
        telemetry.addData("statusRear: ",statusRear);
        telemetry.addData("statusIRx: ",statusIRx);
        telemetry.addData("statusIRy: ",statusIRy);
        telemetry.addData("IRx = ",IRx);
        telemetry.addData("IRy = ",IRy);
        telemetry.addData("rangeX = ",rangeX);
        telemetry.addData("rangeY = ",rangeY);
        telemetry.addData("distLeft = ",distLeft/25.4);
        telemetry.addData("distRight = ",distRight/25.4);
        telemetry.addData("distFront = ",distFront/25.4);
        telemetry.addData("distRear = ",distRear/25.4);
        telemetry.update();
        ----- for debugging use only, to set distance offsets for each sensor ----------
   */


        // actual[0] = IRx;
        // actual[1] = IRy;
        //return actual;
        
    } // END:  method getPosition      
  } //END:  class FourCorners
  
//**********************************************************************
