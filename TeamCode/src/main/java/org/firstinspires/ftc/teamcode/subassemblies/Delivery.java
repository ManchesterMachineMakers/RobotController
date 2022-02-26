package org.firstinspires.ftc.teamcode.subassemblies;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.io.File;

import androidx.annotation.RequiresApi;

/**
 * Delivery mechanism for Freight Frenzy
 * Drawer slide to extend to the proper height
 * Grabber to keep or release the freight.
 *
 */
public class Delivery implements Subassembly {

    /**
     * Delivery State - to be written to a file
     */
    public class DeliveryState {
        public int slideHighPosition;
        public int slideMidPosition;
        public int slideLowPosition;
        public int slideHomePosition;

        public double chuteServoLeftCompactPosition;
        public double chuteServoLeftOpenPosition;
        public double chuteServoRightCompactPosition;
        public double chuteServoRightOpenPosition;

        public double doorServoClosedPosition;
        public double doorServoOpenPosition;

        public DeliveryState(){}
    }

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

        String filename = RobotConfig.CURRENT.getValue("calibrationFile_Delivery");
        File file = AppUtil.getInstance().getSettingsFile(filename);
        //state = ReadWriteFile.readFile(file);
        RobotLog.i("retrieved calibration from '%s'", filename);

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
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runSlideToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SLIDE_POWER);
    }

    public void setChuteOpenPosition() {
        chuteServoLeft.setPosition(CHUTE_OPEN_POSITION);
        chuteServoRight.setPosition(CHUTE_OPEN_POSITION);
    }
    public void setChuteCompactPosition() {
        chuteServoLeft.setPosition(CHUTE_COMPACT_POSITION);
        chuteServoRight.setPosition(CHUTE_COMPACT_POSITION);
    }
    public void setDoorClosedPosition() {
        doorServo.setPosition(DOOR_CLOSED_POSITION);
    }
    public void setDoorOpenPosition() {
        doorServo.setPosition(DOOR_OPEN_POSITION);
    }
    public void incrementSlideUp() {
        runSlideToPosition(motor.getCurrentPosition() + SLIDE_INCREMENT);
    }
    public void incrementSlideDown() {
        runSlideToPosition(motor.getCurrentPosition() - SLIDE_INCREMENT);
    }
    // we will likely need to add a tolerance to this method.
    public boolean isFoldedUp() {
        if (chuteServoLeft.getPosition() == CHUTE_COMPACT_POSITION) { return true; }
        return false;
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
    public void controller() {
        // set the slide height
        if (!motor.isBusy()) {
            if (gamepad.a) {
                runSlideToPosition(Delivery.SLIDE_HOME_POSITION);
            } else if (gamepad.x) {
                runSlideToPosition(Delivery.SLIDE_LOW_POSITION);
            } else if (gamepad.y) {
                runSlideToPosition(Delivery.SLIDE_MID_POSITION);
            } else if (gamepad.b) {
                runSlideToPosition(Delivery.SLIDE_HIGH_POSITION);
            } else if (gamepad.dpad_down) {
                incrementSlideDown();
            } else if (gamepad.dpad_up) {
                incrementSlideUp();
            }
        }
        // open and close door with left and right dpad buttons
        if (gamepad.dpad_left) {
            setDoorClosedPosition();
        } else if (gamepad.dpad_right) {
            setDoorOpenPosition();
        }

        // fold and unfold the chute with the back button as a toggle
        if (gamepad.back) {
            if (isFoldedUp()) {
                setChuteOpenPosition();
            } else {
                setChuteCompactPosition();
            }
        }
    }
}
