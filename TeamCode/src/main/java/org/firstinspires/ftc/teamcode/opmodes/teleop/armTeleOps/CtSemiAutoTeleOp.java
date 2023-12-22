// Package and imports
package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm;
import org.firstinspires.ftc.teamcode.util.BaseTeleOp;

@TeleOp(name = "Semi-Auto Ct Arm TeleOp", group = "arm")
public class CtSemiAutoTeleOp extends BaseTeleOp {

    public CtSemiAutoArm semiAutoArm = new CtSemiAutoArm(this);
    public CtSemiAutoTeleOp() { super.arm = semiAutoArm; }

}
