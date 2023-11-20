package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.SemiAutoArm;

// Comments courtesy of ChatGPT
@TeleOp(name = "Semi-Auto TeleOp", group = "arm")
public class SemiAutoTeleOp extends OpMode {

    // Initializing robot subassemblies
    Drivebase drivebase = new Drivebase();
    SemiAutoArm semiAutoArm = new SemiAutoArm();

    @Override
    public void init() {

        // Setting up references for Drivebase
        drivebase._gamepad = gamepad1;
        drivebase._telemetry = telemetry;
        drivebase._hardwareMap = hardwareMap;

        // Setting up references for SemiAutoArm
        semiAutoArm._gamepad = gamepad2;
        semiAutoArm._telemetry = telemetry;
        semiAutoArm._hardwareMap = hardwareMap;

        // Initializing subassemblies
        drivebase.init();
        semiAutoArm.init();
    }

    @Override
    public void loop() {

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