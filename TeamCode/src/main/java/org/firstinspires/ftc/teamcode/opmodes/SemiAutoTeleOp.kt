package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.DroneLauncher
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Winch

@TeleOp(name = "Semi-Automatic TeleOp", group = "arm")
class SemiAutoTeleOp: LinearOpMode() {

    init { telemetry.isAutoClear = false } // ensure subassembly ready messages aren't cleared

    // misc
    val loopTime = ElapsedTime()
    // subassemblies
    val driveBase = DriveBase(this)
    val arm = Arm(this)
    val pixelReleases = PixelReleases(this)
    val winch = Winch(this)
    val droneLauncher = DroneLauncher(this)

    val subassemblyList = listOf(driveBase, arm, pixelReleases, winch, droneLauncher)
    var subassemblyStatuses = "unknown"
        set(value) {
            for (subassembly in subassemblyList) subassembly.status = value
            field = value
        }

    override fun runOpMode() {
        // init, no movement allowed
        subassemblyStatuses = "init"

        runAllTelemetries()
        telemetry.update()

        waitForStart()

        if (opModeIsActive()) {
            // start
            subassemblyStatuses = "start"
//            arm.overcurrentProtection.start()
            telemetry.isAutoClear = true
            while (opModeIsActive()) {
                subassemblyStatuses = "loop"
                loopTime.reset()

                // Subassembly control
                driveBase.control(gamepad1)
                pixelReleases.control(gamepad2)
                winch.control(gamepad2.right_stick_y)
                droneLauncher.control(gamepad2.x)
                arm.control(gamepad2)

                runAllTelemetries()
                telemetry.update()
            }
        }
    }

    fun telemetry() {
        telemetry.addData("runtime (s)", runtime)
        telemetry.addData("looptime (ms)", loopTime.milliseconds())
        runAllTelemetries()
    }

    fun runAllTelemetries() {
        for (subassembly in subassemblyList) subassembly.telemetry()
        telemetry.update()
    }
}