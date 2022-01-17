package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.util.Names;

/**
 * Intake should be mounted with the motor at the top, facing the __ of the robot.
 * This should work automatically if possible. How should we detect rings to be taken in?
 * It may not be practical to detect rings, but instead to have this managed by the driver.
 * 312 rpm motor. 4" wheels.
 */
public class ActiveIntake implements Testable {

    boolean stop = false;
    double intakePower = 0.5;
    DcMotor motor;
    TouchSensor ringSensor;

    OpMode opMode;
    double timeLastRingTaken;
    public static int maxRingsAllowedOnBot = 1;

    /**
     * Pass in the hardware map in the constructor in order to get the motor.
     * @param opMode pass in the OpMode.
     */
    public ActiveIntake(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        motor = hardwareMap.get(DcMotor.class, Names.motor_Intake);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're just turning the motor on or off
        motor.setPower(0);
        ((DcMotorEx)motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Turn the intake on in order to fetch rings
     * @return boolean success
     */
    public boolean go(DcMotorSimple.Direction direction) {
        stop = false;
        try {
            motor.setDirection(direction);
            motor.setPower(intakePower);

            // figure out if we have a ring, and if it made it all the way up the intake?


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
            float rotations1 = deltaPos1/(float)Shooter.motorEncoderEventsPerRotation; // how many rotations
            float rpm1 = (rotations1/ms) * 60000; // rotations per minute.
            return rpm1;

        } catch (InterruptedException ex) {
            RobotLog.logStackTrace(ex);
        }
        return 0;
    }
}
