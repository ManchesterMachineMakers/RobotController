// Package and imports
package org.firstinspires.ftc.teamcode.opmodes.teleop.semiAutoArms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto.CtSemiAutoArm;

// Comments courtesy of ChatGPT
@TeleOp(name = "Semi-Auto Ct Arm TeleOp", group = "arm")
public class CtSemiAutoTeleOp extends OpMode {

    // Initializing robot subassemblies
    Drivebase drivebase = new Drivebase();
    CtSemiAutoArm semiAutoArm = new CtSemiAutoArm();

    @Override
    public void init() {
        // Update statuses
        drivebase.currentStatus = "initializing";
        semiAutoArm.currentStatus = "initializing";

        // Setting up references for Drivebase
        drivebase.gamepad = gamepad1;
        drivebase.telemetry = telemetry;
        drivebase.hardwareMap = hardwareMap;

        // Setting up references for SemiAutoArm
        semiAutoArm.gamepad = gamepad2;
        semiAutoArm.telemetry = telemetry;
        semiAutoArm.hardwareMap = hardwareMap;

        // Initializing subassemblies
        drivebase.init();
        semiAutoArm.init();

        // Update telemetry
        drivebase.telemetry();
        semiAutoArm.telemetry();
    }

    @Override
    public void loop() {
        // Update statuses
        drivebase.currentStatus = "looping";
        semiAutoArm.currentStatus = "looping";

        // Update run time
        drivebase.runTime = time;
        semiAutoArm.runTime = time;

        // Safety check: Stops the robot if SemiAutoArm requests it
        if (semiAutoArm.needsStop) {
            requestOpModeStop();
        } else if (semiAutoArm.arm.getCurrent(CurrentUnit.AMPS) > 10) {
            // Emergency stop if arm current exceeds 10 Amps
            terminateOpModeNow();
        }

        // Main loop functions for Drivebase and SemiAutoArm
        drivebase.loop();
        semiAutoArm.loop();

        // Telemetry updates for monitoring
        drivebase.telemetry();
        semiAutoArm.telemetry();
    }
}
