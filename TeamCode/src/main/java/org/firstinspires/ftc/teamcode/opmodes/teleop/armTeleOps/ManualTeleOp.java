package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.ManualArm;

// Comments courtesy of ChatGPT
@TeleOp(name = "Manual Arm TeleOp", group = "arm")
public class ManualTeleOp extends OpMode {

    // Initializing robot subassemblies
    Drivebase drivebase = new Drivebase(this);
    ManualArm manualArm = new ManualArm(this);

    @Override
    public void init() {
        // Initializing subassemblies
        drivebase.init();
        manualArm.init();
    }

    @Override
    public void loop() {

        // Safety check: Stops the robot if ManualArm requests it
        if (manualArm.needsStop) {
            requestOpModeStop();
        } else if (manualArm.arm.getCurrent(CurrentUnit.AMPS) > 8) {
            // Emergency stop if arm current exceeds 10 Amps
            terminateOpModeNow();
        }

        // Main loop functions for Drivebase and ManualArm
        drivebase.loop();
        manualArm.loop();

        // Telemetry updates for monitoring
        drivebase.telemetry();
        manualArm.telemetry();
    }
}