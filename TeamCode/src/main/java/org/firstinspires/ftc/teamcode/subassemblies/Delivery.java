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

    private static final double CHUTE_COMPACT_POSITION = 0;
    private static final double CHUTE_OPEN_POSITION = 0.5;
    public static final int SLIDE_HOME_POSITION = 0;
    public static final int SLIDE_LOW_POSITION = 200;
    public static final int SLIDE_MID_POSITION = 500;
    public static final int SLIDE_HIGH_POSITION = 800;
    public static final int SLIDE_CAP_POSITION = 1000;
    private static final double DOOR_CLOSED_POSITION = 0;
    private static final double DOOR_OPEN_POSITION = 0.5;
    private static final int SLIDE_INCREMENT = 100;
    private static final double SLIDE_POWER = 0.5;
    // values for telemetry
    private int motorPosition;
    private DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    private double chuteServoLeftPosition;
    private double chuteServoRightPosition;
    private double doorServoPosition;
    private Servo.Direction chuteServoLeftDirection = Servo.Direction.FORWARD;
    private Servo.Direction chuteServoRightDirection = Servo.Direction.REVERSE;

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
    // we will likely need to add a tolerance to this method.
    public boolean isFoldedUp() {
        double servoTolerance = 0.25;
        double chuteCurrentPosition = chuteServoLeft.getPosition();
        return chuteCurrentPosition >= state.chuteServoLeftCompactPosition - servoTolerance || chuteCurrentPosition >= state.chuteServoLeftCompactPosition + servoTolerance;
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
            while (gamepad.a || gamepad.b || gamepad.x || gamepad.y) opMode.idle();
        }
        if (gamepad.dpad_down) {
            incrementSlideDown();
            while (gamepad.dpad_down) opMode.idle();
        } else if (gamepad.dpad_up) {
            incrementSlideUp();
            while (gamepad.dpad_up) opMode.idle();
        }
        // open and close door with left and right dpad buttons
        if (gamepad.dpad_left) {
            setDoorClosedPosition();
            while (gamepad.dpad_left) opMode.idle();
        } else if (gamepad.dpad_right) {
            setDoorOpenPosition();
            while (gamepad.dpad_right) opMode.idle();
        }

        // fold and unfold the chute with the back button as a toggle
        if (gamepad.back) {
            if (isFoldedUp()) {
                setChuteOpenPosition();
            } else {
                setChuteCompactPosition();
            }
            while (gamepad.back) opMode.idle();
        }
    }

    /**
     * Add useful telemetry whenever we use this subassembly
     */
    public void composeTelemetry(Telemetry telemetry) {

        // At the beginning of each telemetry update, grab a bunch of data
        // that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            motorPosition = motor.getCurrentPosition();
            chuteServoLeftPosition = chuteServoLeft.getPosition();
            chuteServoRightPosition = chuteServoRight.getPosition();
            doorServoPosition = doorServo.getPosition();
        }
        });

        telemetry.addLine("Current Slide Position").addData("Motor", motorPosition);
        telemetry.addLine("Current Chute Servo Positions").addData("Left", chuteServoLeftPosition )
                .addData("Right", chuteServoRightPosition );
        telemetry.addLine("Current Door Servo Position").addData( "Door", doorServoPosition );
    }
}
