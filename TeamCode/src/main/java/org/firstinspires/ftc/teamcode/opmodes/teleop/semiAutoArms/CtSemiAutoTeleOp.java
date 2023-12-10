// Package and imports
package org.firstinspires.ftc.teamcode.opmodes.teleop.semiAutoArms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.semiAuto.CtSemiAutoArm;
import org.manchestermachinemakers.easyop.Linear;

// Comments courtesy of ChatGPT
@TeleOp(name = "Semi-Auto Ct Arm TeleOp", group = "arm")
public class CtSemiAutoTeleOp extends Linear {

    // Initializing robot subassemblies
    Drivebase drivebase = new Drivebase(this);
    CtSemiAutoArm semiAutoArm = new CtSemiAutoArm(this);

    @Override
    public void opInit() {
        // Update statuses
        drivebase.currentStatus = "initializing";
        semiAutoArm.currentStatus = "initializing";

        // Initializing subassemblies
        drivebase.init();
        semiAutoArm.init();

        // Update telemetry
        drivebase.telemetry();
        semiAutoArm.telemetry();
    }

    @Override
    public void opLoop() {
        // Update statuses
        drivebase.currentStatus = "looping";
        semiAutoArm.currentStatus = "looping";

        // Update run time
        drivebase.runTime = time;
        semiAutoArm.runTime = time;

        // Protect the arm if it's overcurrent (make it not hurt itself)
        semiAutoArm.overcurrentProtection();

        // Main loop functions for Drivebase and SemiAutoArm
        drivebase.loop();
        semiAutoArm.loop();

        // Telemetry updates for monitoring
        drivebase.telemetry();
        semiAutoArm.telemetry();
    }
}
