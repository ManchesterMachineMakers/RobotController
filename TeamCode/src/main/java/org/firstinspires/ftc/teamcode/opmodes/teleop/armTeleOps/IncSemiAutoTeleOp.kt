package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.IncSemiAutoArm
import org.firstinspires.ftc.teamcode.util.BaseTeleOp

@TeleOp(name = "Semi-Auto Inc Arm TeleOp", group = "arm")
class IncSemiAutoTeleOp : OpMode() {
    // Initializing robot subassemblies
    private val driveBase = DriveBase(this, gamepad1)
    private val incSemiAutoArm = IncSemiAutoArm(this, gamepad2)
    private val base = BaseTeleOp(incSemiAutoArm, driveBase)

    override fun init() = base.init()
    override fun start() = base.start()
    override fun loop() = base.loop()
}