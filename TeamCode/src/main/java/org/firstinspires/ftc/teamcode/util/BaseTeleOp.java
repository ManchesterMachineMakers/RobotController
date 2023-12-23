package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;

public class BaseTeleOp {

    public BaseArm arm;
    public Drivebase drivebase;

    public BaseTeleOp(BaseArm arm, Drivebase drivebase) {
        this.arm = arm;
        this.drivebase = drivebase;
    }

    public void init() {
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

    public void start() {
        arm.start();
    }

    public void loop() {
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
