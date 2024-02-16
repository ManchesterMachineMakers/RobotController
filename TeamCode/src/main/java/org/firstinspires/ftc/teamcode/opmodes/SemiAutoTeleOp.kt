package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Winch

@TeleOp(name = "Semi-Automatic TeleOp", group = "arm")
class SemiAutoTeleOp: LinearOpMode() {

    val driveBase = DriveBase(this)
    val arm = Arm(this)
    val pixelReleases = PixelReleases(this)
    val winch = Winch(this)

    val subassemblyList = listOf(driveBase, arm, pixelReleases, winch)

    override fun runOpMode() {
        // init, no movement allowed
        telemetry.isAutoClear = false

        runAllTelemetries()

        waitForStart()

        if (opModeIsActive()) {
            // start
            while (opModeIsActive()) {
                driveBase.control(gamepad1)
                //arm.control(gamepad2)
                pixelReleases.control(gamepad2)
                winch.control(gamepad2)

                runAllTelemetries()
            }
        }
    }

    fun runAllTelemetries() {
        for (subassembly in subassemblyList) subassembly.telemetry()
        telemetry.update()
    }
}