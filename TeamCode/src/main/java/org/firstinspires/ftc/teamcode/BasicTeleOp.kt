package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.contracts.Controllable.Companion.runAll
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase

// Imports Arm subassembly
// Imports DriveBase subassembly
// Imports Inject
// Imports Linear
@TeleOp(name = "Basic TeleOp")
class BasicTeleOp : LinearOpMode() {
    override fun runOpMode() {
        // grab subassemblies
        val driveBase = DriveBase(this)
        val arm = Arm(this)
        waitForStart()
        while(opModeIsActive() && !isStopRequested) {
            driveBase.loop(gamepad1)
        }
    }
}
