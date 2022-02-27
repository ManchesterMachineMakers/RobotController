package org.firstinspires.ftc.teamcode.subassemblies;

import android.os.Build;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.io.File;
import java.io.IOException;

import androidx.annotation.RequiresApi;

import station.util.Persist;

/**
 * Delivery mechanism for Freight Frenzy
 * Drawer slide to extend to the proper height
 * Grabber to keep or release the freight.
 *
 */
public class Delivery implements Subassembly {

    public static DeliveryState state;

    public static final double MOTOR_ENCODERS_PER_ROTATION = 1425.1;
    private static final double CHUTE_COMPACT_POSITION = 0;
    private static final double CHUTE_OPEN_POSITION = 0.5;
    public static final int SLIDE_HOME_POSITION = 0;
    public static final int SLIDE_LOW_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 0.75);
    public static final int SLIDE_MID_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 1.5);
    public static final int SLIDE_HIGH_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 3);
    public static final int SLIDE_CAP_POSITION = SLIDE_HIGH_POSITION;
    private static final double DOOR_CLOSED_POSITION = 0;
    private static final double DOOR_OPEN_POSITION = 0.5;
    private static final int SLIDE_INCREMENT = (int)(MOTOR_ENCODERS_PER_ROTATION/10);
    private static final double SLIDE_POWER = 0.5;
    // values for telemetry
    public int motorPosition;
    public static final DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    public double chuteServoLeftPosition;
    public double chuteServoRightPosition;
    public double doorServoPosition;
    public static final Servo.Direction chuteServoLeftDirection = Servo.Direction.FORWARD;
    public static final Servo.Direction chuteServoRightDirection = Servo.Direction.REVERSE;

    /**
     * Delivery State - to be written to a file
     */
    public class DeliveryState {
        public int slideHighPosition = SLIDE_HIGH_POSITION;
        public int slideMidPosition = SLIDE_MID_POSITION;
        public int slideLowPosition = SLIDE_LOW_POSITION;
        public int slideHomePosition = SLIDE_HOME_POSITION;

        public double chuteServoLeftCompactPosition = CHUTE_COMPACT_POSITION;
        public double chuteServoLeftOpenPosition = CHUTE_OPEN_POSITION;

        public double chuteServoRightCompactPosition = CHUTE_COMPACT_POSITION;
        public double chuteServoRightOpenPosition = CHUTE_OPEN_POSITION;


        public double doorServoClosedPosition = DOOR_CLOSED_POSITION;
        public double doorServoOpenPosition = DOOR_OPEN_POSITION;

        public boolean runPastLimits = false;
        public DeliveryState(){}
    }

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
        if (motor.isBusy() && position == motor.getTargetPosition()) return;
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SLIDE_POWER);
    }

    public void setChuteOpenPosition() {
        chuteServoLeft.setPosition(state.chuteServoLeftOpenPosition);
        chuteServoRight.setPosition(state.chuteServoRightOpenPosition);
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
        runSlideToPosition(motor.getCurrentPosition() + SLIDE_INCREMENT);
    }
    public void incrementSlideDown() {
        runSlideToPosition(motor.getCurrentPosition() - SLIDE_INCREMENT);
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
        if (!motor.isBusy()) {
            if (gamepad.a) {
                runSlideToPosition(state.slideHomePosition);
            } else if (gamepad.x) {
                runSlideToPosition(state.slideLowPosition);
            } else if (gamepad.y) {
                runSlideToPosition(state.slideMidPosition);
            } else if (gamepad.b) {
                runSlideToPosition(state.slideHighPosition);
            }
        }
        if (gamepad.dpad_down) {
            incrementSlideDown();
        } else if (gamepad.dpad_up) {
            incrementSlideUp();
        }
        // open and close door with left and right dpad buttons
        if (gamepad.dpad_left) {
            setDoorClosedPosition();
        } else if (gamepad.dpad_right) {
            setDoorOpenPosition();
        }

        // fold and unfold the chute with the back button as a toggle
        if (gamepad.back) {
            if (chuteServoLeft.getPosition() < CHUTE_OPEN_POSITION) {
                setChuteOpenPosition();
            } else {
                setChuteCompactPosition();
            }
            while (gamepad.back) opMode.idle();
        }

        motorPosition = motor.getCurrentPosition();
        chuteServoLeftPosition = chuteServoLeft.getPosition();
        chuteServoRightPosition = chuteServoRight.getPosition();
        doorServoPosition = doorServo.getPosition();
    }

}
