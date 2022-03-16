package org.firstinspires.ftc.teamcode.subassemblies;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.io.File;
import java.io.IOException;

import station.util.Persist;

/**
 * Delivery mechanism for Freight Frenzy
 * Drawer slide to extend to the proper height
 * Grabber to keep or release the freight.
 *
 */
public class Delivery implements Subassembly {

    private boolean isWithinTolerance(double a, double b) {
        return Math.abs(a - b) < 0.02;
    }

    public static DeliveryState state;
  
    public static final double MOTOR_ENCODERS_PER_ROTATION = 1425.1;
    public static final int SLIDE_HOME_POSITION = 0;
    public static final int SLIDE_LOW_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 0.75);
    public static final int SLIDE_MID_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 2);
    public static final int SLIDE_HIGH_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 3.5);
    public static final int SLIDE_CAP_POSITION = SLIDE_HIGH_POSITION;
    private static final double DOOR_CLOSED_POSITION = 5/300.0;
    private static final double DOOR_OPEN_POSITION = 150/300.0;
    private static final int SLIDE_INCREMENT = (int)(MOTOR_ENCODERS_PER_ROTATION/10);
    private static final double SLIDE_POWER = 0.5;
    // values for telemetry
    public int motorPosition;
    public static final DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    public double chuteServoLeftPosition;
    public double chuteServoRightPosition;
    public double doorServoPosition;
    public static final Servo.Direction chuteServoLeftDirection = Servo.Direction.REVERSE;
    public static final Servo.Direction chuteServoRightDirection = Servo.Direction.REVERSE;

    /**
     * Delivery State - to be written to a file
     */
    public class DeliveryState {
        public int slideHighPosition = SLIDE_HIGH_POSITION;
        public int slideMidPosition = SLIDE_MID_POSITION;
        public int slideLowPosition = SLIDE_LOW_POSITION;
        public int slideHomePosition = SLIDE_HOME_POSITION;

        // this goes backwards from what it should be
        public double chuteServoLeftBasePosition = 2/300.0;
        public double chuteServoLeftCompactPosition = chuteServoLeftBasePosition + 195/300.0;
        public double chuteServoLeftHomePosition = chuteServoLeftBasePosition + 92/300.0;
        public double chuteServoLeftDeliverPosition = chuteServoLeftBasePosition + 60/300.0;

        public double chuteServoRightBasePosition = 1 - 1/300.0;
        public double chuteServoRightCompactPosition = chuteServoRightBasePosition - 195/300.0;
        public double chuteServoRightHomePosition = chuteServoRightBasePosition - 92/300.0;
        public double chuteServoRightDeliverPosition = chuteServoRightBasePosition - 60/300.0;

        public double doorServoClosedPosition = DOOR_CLOSED_POSITION;
        public double doorServoOpenPosition = DOOR_OPEN_POSITION;

        public boolean runPastLimits = false;

        public DeliveryState() {
        }
    }
    private static final double CHUTE_ADJUSTMENT_ANGLE = 90-78.7;
    // switch + and - by using a button on the controller?
    private static boolean REVERSE_CHUTE_ADJUSTMENT = false;

    private boolean wasSlideHomeRequested;
    private boolean wasSlideLowRequested;
    private boolean wasSlideMidRequested;
    private boolean wasSlideHighRequested;
    private boolean wasDoorOpenPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public Delivery(OpMode opMode) {
        chuteServoLeft = opMode.hardwareMap.servo.get(RobotConfig.CURRENT.name("servo_DeliveryChuteLeft"));
        chuteServoRight = opMode.hardwareMap.servo.get(RobotConfig.CURRENT.name("servo_DeliveryChuteRight"));
        doorServo = opMode.hardwareMap.servo.get(RobotConfig.CURRENT.name("servo_DeliveryDoor"));
        motor = opMode.hardwareMap.dcMotor.get(RobotConfig.CURRENT.name("motor_DeliveryMotor"));

        if (RobotConfig.CURRENT.name("delivery_Gamepad").equals("gamepad2")) {
            gamepad = opMode.gamepad2; // default value
        } else {
            gamepad = opMode.gamepad1;
        }

        String filename = RobotConfig.CURRENT.getValue("deliveryCalibrationFile");
        File file = AppUtil.getInstance().getSettingsFile(filename);
        try {
            state = Persist.readFromFile(file.getAbsolutePath());
        } catch (ClassNotFoundException | IOException e) {
            e.printStackTrace();
            state = new DeliveryState();
        }
        RobotLog.i("retrieved calibration from '%s'", filename);
        RobotLog.i("Values: %s", new Gson().toJson(state));

        zero();
    }

    /**
     * These can each be set directly, but you can also use the shortcuts.
     */
    public Servo chuteServoLeft;
    public Servo chuteServoRight;
    public Servo doorServo;
    public DcMotor motor;
    public Gamepad gamepad;

    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(motorDirection);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        chuteServoLeft.setDirection(chuteServoLeftDirection);
        chuteServoRight.setDirection(chuteServoRightDirection);
    }

    public void runSlideToPosition(int position) {
        if (!state.runPastLimits && (position > state.slideHighPosition || position < state.slideHomePosition)) return;
        if (motor.isBusy()) {
            if (motor.getPower() == 0) {
                motor.setPower(SLIDE_POWER);
            } else {
                return;
            }
        }
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SLIDE_POWER);
    }

    public void setChuteDeliverPosition() {
        chuteServoLeft.setPosition(state.chuteServoLeftDeliverPosition);
        chuteServoRight.setPosition(state.chuteServoRightDeliverPosition);
    }
    public void setChuteHomePosition() {
        chuteServoLeft.setPosition(state.chuteServoLeftHomePosition);
        chuteServoRight.setPosition(state.chuteServoRightHomePosition);
    }
    public void setChuteCompactPosition() {
        chuteServoLeft.setPosition(state.chuteServoLeftCompactPosition);
        chuteServoRight.setPosition(state.chuteServoRightCompactPosition);
    }
    public void setDoorClosedPosition() {
        doorServo.setPosition(state.doorServoClosedPosition);
    }
    public void setDoorOpenPosition() {
        doorServo.setPosition(state.doorServoOpenPosition);
    }
    public void incrementSlideUp() {
        int proposed = motor.getCurrentPosition() + SLIDE_INCREMENT;
        if (proposed > SLIDE_CAP_POSITION) { proposed = SLIDE_CAP_POSITION; }
        runSlideToPosition(proposed);
    }
    public void incrementSlideDown() {
        int proposed = motor.getCurrentPosition() - SLIDE_INCREMENT;
        if (proposed < SLIDE_HOME_POSITION) { proposed = SLIDE_HOME_POSITION; }
        runSlideToPosition(proposed);
    }
    // we will likely need to add a tolerance to this method.
    public boolean isDoorClosed() {
        return doorServo.getPosition() < state.doorServoOpenPosition - 0.1;
    }

    /**
     *         Default GamePad Controls
     *         A - “home” position: chute at 30 degrees, resting on drivebase
     *         X - level 1 delivery
     *         Y - level 2 delivery
     *         B - level 3 delivery
     *         D-pad up/down: override/precision control of chute height
     *         D-pad left: close door
     *         D-pad right: open door
     *         back - toggle chute to compact or open position
     */
    public void controller(LinearOpMode opMode) {
        // set the slide height
        if (wasSlideHomeRequested || wasSlideLowRequested || wasSlideMidRequested || wasSlideHighRequested ) {
            if (wasSlideHomeRequested) {
                runSlideToPosition(state.slideHomePosition);
                wasSlideHomeRequested = false;
            } else if (wasSlideLowRequested) {
                runSlideToPosition(state.slideLowPosition);
                wasSlideLowRequested = false;
            } else if (wasSlideMidRequested) {
                runSlideToPosition(state.slideMidPosition);
                wasSlideMidRequested = false;
            } else if (wasSlideHighRequested) {
                runSlideToPosition(state.slideHighPosition);
                wasSlideHighRequested = false;
            }
        } else if (gamepad.a) {
            wasSlideHomeRequested = true;
        } else if (gamepad.x) {
            wasSlideLowRequested = true;
        } else if (gamepad.y) {
            wasSlideMidRequested = true;
        } else if (gamepad.b) {
            wasSlideHighRequested = true;
        }

        if (gamepad.dpad_down) {
            incrementSlideDown();
        } else if (gamepad.dpad_up) {
            incrementSlideUp();
        }
        // Change the chute position with the dpad
        if (gamepad.dpad_left) {
                wasDpadLeftPressed = true;
        } else if (wasDpadLeftPressed) {
            // close the chute up
            // if it's already compact, do nothing
            if (isWithinTolerance(chuteServoLeftPosition, state.chuteServoLeftCompactPosition)) {
//                setChuteDeliverPosition();
            } else if (isWithinTolerance(chuteServoLeftPosition, state.chuteServoLeftDeliverPosition)) {
                // if it's in the deliver position, move to home
                setChuteHomePosition();
            } else if (isWithinTolerance(chuteServoLeftPosition, state.chuteServoLeftHomePosition)) {
                // if it's in the home position, move to compact
                setChuteCompactPosition();
            } else {
                setChuteHomePosition();
            }

            wasDpadLeftPressed = false;
        }

        if (gamepad.dpad_right) {
                wasDpadRightPressed = true;
        } else if (wasDpadRightPressed) {
            // open the chute down
            // if it's in the compact position, move to home
            if (isWithinTolerance(chuteServoLeftPosition, state.chuteServoLeftCompactPosition)) {
                setChuteHomePosition();
            }
            // if it's in the home position, move to deliver
            else if (isWithinTolerance(chuteServoLeftPosition, state.chuteServoLeftHomePosition)) {
                setChuteDeliverPosition();
            }
            // if it's in the deliver position, do nothing.
            else if (isWithinTolerance(chuteServoLeftPosition, state.chuteServoLeftDeliverPosition)) {
//                setChuteCompactPosition();
            } else {
                setChuteHomePosition();
            }
            wasDpadRightPressed = false;
        }

        // fold and unfold the chute with the back button as a toggle
        if (gamepad.back) {
            wasDoorOpenPressed = true;
        } else {
            if(wasDoorOpenPressed) {
                if (isDoorClosed()) {
                    setDoorOpenPosition();
                } else {
                    setDoorClosedPosition();
                }
            }
            wasDoorOpenPressed = false;
        }

        motorPosition = motor.getCurrentPosition();
        chuteServoLeftPosition = chuteServoLeft.getPosition();
        chuteServoRightPosition = chuteServoRight.getPosition();
        doorServoPosition = doorServo.getPosition();
    }

}
