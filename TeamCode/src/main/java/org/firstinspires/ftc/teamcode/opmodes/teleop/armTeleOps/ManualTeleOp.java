package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.ManualArm;

@TeleOp(name = "Manual Arm TeleOp", group = "arm")
public class ManualTeleOp extends OpMode {

    // Initializing robot subassemblies
    Drivebase drivebase = new Drivebase(this);
    ManualArm manualArm = new ManualArm(this);

    @Override
    public void init() {

        drivebase.setCurrentStatus("initializing");
        manualArm.setCurrentStatus("initializing");

        // Initializing subassemblies
        drivebase.init();
        manualArm.init();

        drivebase.telemetry();
        manualArm.telemetry();
    }

    @Override
    public void loop() {

        drivebase.setCurrentStatus("looping");
        manualArm.setCurrentStatus("looping");

        // Main loop functions for Drivebase and ManualArm
        drivebase.loop();
        manualArm.loop();

        // Telemetry updates for monitoring
        drivebase.telemetry();
        manualArm.telemetry();
    }
}