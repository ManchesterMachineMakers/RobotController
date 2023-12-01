package org.firstinspires.ftc.teamcode.opmodes.teleop.semiAutoArms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto.IncSemiAutoArm;

// Comments courtesy of ChatGPT
@TeleOp(name = "Semi-Auto Inc Arm TeleOp", group = "arm")
public class IncSemiAutoTeleOp extends OpMode {

    // Initializing robot subassemblies
    Drivebase drivebase = new Drivebase();
    IncSemiAutoArm incSemiAutoArm = new IncSemiAutoArm();

    @Override
    public void init() {
        // Update statuses
        drivebase.currentStatus = "initializing";
        incSemiAutoArm.currentStatus = "initializing";

        // Setting up references for Drivebase
        drivebase.gamepad = gamepad1;
        drivebase.telemetry = telemetry;
        drivebase.hardwareMap = hardwareMap;

        // Setting up references for SemiAutoArm
        incSemiAutoArm.gamepad = gamepad2;
        incSemiAutoArm.telemetry = telemetry;
        incSemiAutoArm.hardwareMap = hardwareMap;

        // Initializing subassemblies
        drivebase.init();
        incSemiAutoArm.init();

        // Update telemetry
        drivebase.telemetry();
        incSemiAutoArm.telemetry();
    }

    @Override
    public void loop() {
        // Update statuses
        drivebase.currentStatus = "looping";
        incSemiAutoArm.currentStatus = "looping";

        // Update run time
        drivebase.runTime = time;
        incSemiAutoArm.runTime = time;

        // Safety check: Stops the robot if SemiAutoArm requests it
        if (incSemiAutoArm.needsStop) {
            requestOpModeStop();
        } else if (incSemiAutoArm.arm.getCurrent(CurrentUnit.AMPS) > 10) {
            // Emergency stop if arm current exceeds 10 Amps
            terminateOpModeNow();
        }

        // Main loop functions for Drivebase and SemiAutoArm
        drivebase.loop();
        incSemiAutoArm.loop();

        // Telemetry updates for monitoring
        drivebase.telemetry();
        incSemiAutoArm.telemetry();
    }
}