package org.firstinspires.ftc.teamcode.opmodes.teleop.semiAutoArms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto.IncSemiAutoArm;

// Comments courtesy of ChatGPT
@TeleOp(name = "Semi-Auto Inc Arm TeleOp", group = "arm")
public class IncSemiAutoTeleOp extends OpMode {

    // Initializing robot subassemblies
    Drivebase drivebase = new Drivebase(this);
    IncSemiAutoArm incSemiAutoArm = new IncSemiAutoArm(this);

    @Override
    public void init() {

        // Update statuses
        drivebase.currentStatus = "initializing";
        incSemiAutoArm.currentStatus = "initializing";

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

        // Protect arm from hurting itself
        incSemiAutoArm.overcurrentProtection();

        // Main loop functions for Drivebase and SemiAutoArm
        drivebase.loop();
        incSemiAutoArm.loop();

        // Telemetry updates for monitoring
        drivebase.telemetry();
        incSemiAutoArm.telemetry();
    }
}