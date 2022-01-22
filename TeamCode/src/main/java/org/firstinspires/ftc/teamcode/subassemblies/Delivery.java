package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Subassembly;

/**
 * Delivery mechanism for Freight Frenzy
 * Drawer slide to extend to the proper height
 * Grabber to keep or release the freight.
 *
 */
public class Delivery implements Subassembly {

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

    public Delivery(OpMode opMode) {
        chuteServo = opMode.hardwareMap.servo.get(RobotConfig.CURRENT.name("servo_DeliverySlide"));
        doorServo = opMode.hardwareMap.servo.get(RobotConfig.CURRENT.name("servo_GrabberClaw"));
        motor = opMode.hardwareMap.dcMotor.get(RobotConfig.CURRENT.name("motor_DeliveryMotor"));
        if (RobotConfig.CURRENT.name("delivery_Gamepad") == "gamepad1") {
            gamepad = opMode.gamepad2; // default value
        } else {
            gamepad = opMode.gamepad1;
        }
    }

    /**
     * These can each be set directly, but you can also use the shortcuts.
     */
    public Servo chuteServo;
    public Servo doorServo;
    public DcMotor motor;
    public Gamepad gamepad;

    public void runSlideToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(SLIDE_POWER);
    }

    public void setChuteOpenPosition() {
        chuteServo.setPosition(CHUTE_OPEN_POSITION);
    }
    public void setChuteCompactPosition() {
        chuteServo.setPosition(CHUTE_COMPACT_POSITION);
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
        if (chuteServo.getPosition() == CHUTE_COMPACT_POSITION) { return true; }
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
