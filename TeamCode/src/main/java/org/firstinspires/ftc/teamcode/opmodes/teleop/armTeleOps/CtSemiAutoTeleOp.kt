package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm
import org.firstinspires.ftc.teamcode.util.BaseTeleOp

@TeleOp(name = "Semi-Auto Ct Arm TeleOp", group = "arm")
class CtSemiAutoTeleOp : OpMode() {
    // Initializing Robot Subassemblies
    private var semiAutoArm = CtSemiAutoArm(this)
    private var drivebase = DriveBase(this, gamepad1)
    private var base = BaseTeleOp(semiAutoArm, drivebase)

    override fun init() = base.init()
    override fun start() = base.start()
    override fun loop() = base.loop()
}