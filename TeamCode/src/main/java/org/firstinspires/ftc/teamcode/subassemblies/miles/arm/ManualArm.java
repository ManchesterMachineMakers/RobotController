package org.firstinspires.ftc.teamcode.subassemblies.miles.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.BaseArm;

/**
 * Manual arm subassembly for controlling arm and wrist movements using gamepad input.
 */
public class ManualArm extends BaseArm {

    public ManualArm(OpMode opMode) {
        super(opMode);
    }

    /**
     * Main loop for controlling the manual arm.
     */
    @Override
    public void loop() {
        loopTime.reset(); // Keep track of time spent in each loop for debugging

        // Wrist control with right stick on gamepad
        double wristPosition = wrist.getPosition();
        wristPosition += gamepad.right_stick_y / 25;

        // Ensure wristPosition stays within limits
        if (wristPosition < 0) {
            wristPosition = 0;
        } else if (wristPosition > 1) {
            wristPosition = 1;
        }

        // Arm movement with left stick on gamepad
        if (gamepad.left_stick_y != 0.0) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(-gamepad.left_stick_y * ARM_SPEED);
            latestArmPosition = arm.getCurrentPosition();
        } else {
            brakeArm();
        }

        // Wrist control with buttons
        if (!buttonWasPressed) {
            if (gamepad.dpad_up) {
                wristPosition -= 0.05;
            } else if (gamepad.dpad_down) {
                wristPosition += 0.05;
            } else if (gamepad.dpad_left) {
                wristPosition += 0.2;
            } else if (gamepad.dpad_right) {
                wristPosition -= 0.2;
            }
        }
        wrist.setPosition(wristPosition);

        // Reset arm position based on gamepad input
        if (gamepad.b) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        handlePixelDroppers();
        handleOvercurrentProtection();

        // For detecting when a button is pressed.
        buttonWasPressed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right;
    }

    /**
     * Displays relevant telemetry information.
     */
    @Override
    public void telemetry() {
        // Telemetry updates
        telemetry.addData("Arm", "");
        telemetry.addData("loop time (nanoseconds)", loopTime.nanoseconds());
        telemetry.addData("arm mode", arm.getMode());
        telemetry.addData("arm target position", arm.getTargetPosition());
        telemetry.addData("arm position", arm.getCurrentPosition());
        telemetry.addData("arm position discrepancy", arm.getCurrentPosition() - arm.getTargetPosition());
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("arm current (amps)", arm.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
    }
}
