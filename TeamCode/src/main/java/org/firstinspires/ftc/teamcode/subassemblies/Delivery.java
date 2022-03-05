package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static final double MOTOR_ENCODERS_PER_ROTATION = 1425.1;
    private static final double CHUTE_COMPACT_POSITION = 0;
    private static final double CHUTE_OPEN_POSITION = 0.4;
    public static final int SLIDE_HOME_POSITION = 0;
    public static final int SLIDE_LOW_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 0.75);
    public static final int SLIDE_MID_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 1.5);
    public static final int SLIDE_HIGH_POSITION = (int)(MOTOR_ENCODERS_PER_ROTATION * 3);
    public static final int SLIDE_CAP_POSITION = SLIDE_HIGH_POSITION;
    private static final double DOOR_CLOSED_POSITION = 0;
    private static final double DOOR_OPEN_POSITION = 0.5;
    private static final int SLIDE_INCREMENT = (int)(MOTOR_ENCODERS_PER_ROTATION/10);
    private static final double SLIDE_POWER = 0.5;

    public static final DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    private static final double CHUTE_ADJUSTMENT_ANGLE = 90-78.7;
    // switch + and - by using a button on the controller?
    private static boolean REVERSE_CHUTE_ADJUSTMENT = false;

    private boolean wasChuteOpenPressed = false;


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
        chuteServoRight.setPosition(1 - CHUTE_OPEN_POSITION);
    }
    public void setChuteCompactPosition() {
        chuteServoLeft.setPosition(CHUTE_COMPACT_POSITION);
        chuteServoRight.setPosition(1 - CHUTE_COMPACT_POSITION);
    }
    public void setDoorClosedPosition() {
        doorServo.setPosition(DOOR_CLOSED_POSITION);
    }
    public void setDoorOpenPosition() {
        doorServo.setPosition(DOOR_OPEN_POSITION);
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
    public boolean isFoldedUp() {
        return chuteServoLeft.getPosition() < CHUTE_OPEN_POSITION -0.1;
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
            wasChuteOpenPressed = true;
        } else {
            if(wasChuteOpenPressed) {
                if (isFoldedUp()) {
                    setChuteOpenPosition();
                } else {
                    setChuteCompactPosition();
                }
            }
            wasChuteOpenPressed = false;
        }
    }
}
