// Package and imports
package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm;
import org.firstinspires.ftc.teamcode.util.BaseTeleOp;

@TeleOp(name = "Semi-Auto Ct Arm TeleOp", group = "arm")
public class CtSemiAutoTeleOp extends OpMode {

    CtSemiAutoArm semiAutoArm = new CtSemiAutoArm(this);
    Drivebase drivebase = new Drivebase(this);
    BaseTeleOp base = new BaseTeleOp(semiAutoArm, drivebase);

    @Override
    public void init() {
        base.init();
    }

    @Override
    public void start() {
        base.start();
    }

    @Override
    public void loop() {
        base.loop();
    }

}
