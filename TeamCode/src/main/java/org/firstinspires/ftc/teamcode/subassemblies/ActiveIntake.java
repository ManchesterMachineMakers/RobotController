package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Subassembly;

/**
 * Intake should be mounted with the motor at the top, facing the __ of the robot.
 * This should work automatically if possible. How should we detect rings to be taken in?
 * It may not be practical to detect rings, but instead to have this managed by the driver.
 * 312 rpm motor. 4" wheels.
 */
public class ActiveIntake implements Subassembly {

    private final Gamepad gamepad;
    public double currentPower;
    boolean stop = false;
    public final double FAST_POWER = 1.0;
    public final double SLOW_POWER_MULTIPLIER = 0.5;
    public static final double motorEncoderEventsPerRotation = 753.2;
    public DcMotor motor;

    OpMode opMode;

    /**
     * Pass in the hardware map in the constructor in order to get the motor.
     * @param opMode pass in the OpMode.
     */
    public ActiveIntake(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        motor = hardwareMap.get(DcMotor.class, RobotConfig.CURRENT.name("motor_Intake"));
        initIntakeMotor();

        if (RobotConfig.CURRENT.name("delivery_Gamepad").equals("gamepad2")) {
            gamepad = opMode.gamepad2; // default value
        } else {
            gamepad = opMode.gamepad1;
        }
    }

    public void initIntakeMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're just turning the motor on or off
        motor.setPower(0);
    }

    /**
     * Turn the intake on in order to fetch rings
     * @return boolean success
     */
    public boolean go(DcMotorSimple.Direction direction) {
        return go(direction, FAST_POWER);
    }

    public boolean go(DcMotorSimple.Direction direction, double power) {
        stop = false;
        try {
            motor.setDirection(direction);
            motor.setPower(power);

        } catch (Exception ex) {
            return false;
        }
        return true;
    }

    /**
     * Stop the intake
     * @return is there a ring still in the intake? True=yes, False=no.
     */
    public boolean stop() {
        stop = true;
        motor.setPower(0);
        // figure out if we have a ring?
        return false;
    }

    /**
     * Get measured speed by encoders.
     * @return RPM
     */
    public float getSpeed() {
        try {
            // measure RPMs using encoders.
            ElapsedTime time = new ElapsedTime();
            time.reset();
            int startPos1 = motor.getCurrentPosition();
            Thread.sleep(500);
            float ms = (float)time.milliseconds();
            int endPos1 = motor.getCurrentPosition();

            float deltaPos1 = endPos1 - startPos1;
            // it's the same kind of  motor as the shooter uses.
            float rotations1 = deltaPos1/(float)motorEncoderEventsPerRotation; // how many rotations
            return (rotations1/ms) * 60000;

        } catch (InterruptedException ex) {
            RobotLog.logStackTrace(ex);
        }
        return 0;
    }

    /**
     * Default controls
     *         Intake
     *         RB - Take in
     *         RT - Take in (slow)
     *         LB - Push out
     *         LT - Push out (slow)
     */
    public void controller() {
//        if (gamepad.right_bumper) {
//            go(DcMotorSimple.Direction.FORWARD, FAST_POWER);
//        } else if (gamepad.left_bumper) {
//            go(DcMotorSimple.Direction.REVERSE, FAST_POWER);
//        } else
        if (gamepad.right_trigger > 0) {
            go(DcMotorSimple.Direction.FORWARD, SLOW_POWER_MULTIPLIER * gamepad.right_trigger);
        } else if (gamepad.left_trigger > 0) {
            go(DcMotorSimple.Direction.REVERSE, SLOW_POWER_MULTIPLIER * gamepad.left_trigger);
        }
        if ((!gamepad.right_bumper) && (!gamepad.left_bumper) && (gamepad.right_trigger + gamepad.left_trigger == 0)) {
            stop();
        }

        currentPower = motor.getPower();
    }
}
