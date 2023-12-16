// Package and imports
package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm;
import org.manchestermachinemakers.easyop.Linear;

@TeleOp(name = "Semi-Auto Ct Arm TeleOp", group = "arm")
public class CtSemiAutoTeleOp extends Linear {

    // Initializing robot subassemblies
    public Drivebase drivebase = new Drivebase(this);
    public CtSemiAutoArm semiAutoArm = new CtSemiAutoArm(this);

    @Override
    public void opInit() {
        drivebase.setCurrentStatus("initializing");
        semiAutoArm.setCurrentStatus("initializing");

        drivebase.init();
        semiAutoArm.init();

        drivebase.telemetry();
        semiAutoArm.telemetry();
    }

    @Override
    public void opLoop() {
        drivebase.setCurrentStatus("looping");
        semiAutoArm.setCurrentStatus("looping");

        drivebase.init();
        semiAutoArm.init();

        drivebase.telemetry();
        semiAutoArm.telemetry();
    }

}
