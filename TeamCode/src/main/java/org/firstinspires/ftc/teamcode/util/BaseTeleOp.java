package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.manchestermachinemakers.easyop.Linear;

public abstract class BaseTeleOp extends Linear {

    public BaseArm arm;
    public Drivebase drivebase = new Drivebase(this);

    public void opInit() {
        // Update statuses
        drivebase.setCurrentStatus("initializing");
        arm.setCurrentStatus("initializing");

        // Initialize
        drivebase.init();
        arm.init();

        // Update telemetry
        drivebase.telemetry();
        arm.telemetry();
    }

    public void opLoop() {
        // Update statuses
        drivebase.setCurrentStatus("looping");
        arm.setCurrentStatus("looping");

        // Loops
        drivebase.loop();
        arm.loop();

        // Update telemetry
        drivebase.telemetry();
        arm.telemetry();
    }
}
