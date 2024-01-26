package org.firstinspires.ftc.teamcode.opmodes.teleop.armTeleOps

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm
import org.firstinspires.ftc.teamcode.util.bases.BaseArmTeleOp

@TeleOp(name = "Semi-Auto Ct Arm TeleOp (Preferred)", group = "arm")
class CtSemiAutoTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val driveBase = DriveBase(this)
        val semiAutoArm = CtSemiAutoArm(this)
        val baseTeleOp = BaseArmTeleOp(this, driveBase, semiAutoArm)

        baseTeleOp.init()
        baseTeleOp.telemetry()

        waitForStart()

        if (opModeIsActive()) {

            baseTeleOp.start()
            baseTeleOp.telemetry()

            while (opModeIsActive()) {
                baseTeleOp.loop()
                baseTeleOp.telemetry()
            }
        }
    }
}