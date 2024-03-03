package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.DroneLauncher
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Winch
import org.firstinspires.ftc.teamcode.util.log

@TeleOp(name = "Semi-Automatic TeleOp", group = "arm")
class SemiAutoTeleOp: LinearOpMode() {

    init {
        telemetry.isAutoClear = false
    } // ensure subassembly ready messages aren't cleared

    override fun runOpMode() {
        // init, no movement allowed

        val driveBase = DriveBase(this)
        val arm = Arm(this)
        val pixelReleases = PixelReleases(this)
        val winch = Winch(this)
        val droneLauncher = DroneLauncher(this)

        val loopTime = ElapsedTime()

        val subassemblyList = listOf(driveBase, arm, pixelReleases, winch, droneLauncher)

        runAllTelemetries()
        telemetry.update()


        waitForStart()

        if (opModeIsActive()) {
            // start
//            arm.overcurrentProtection.start()
            telemetry.isAutoClear = true
            while (opModeIsActive()) {
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
//        telemetry.addData("runtime (s)", runtime)
//        telemetry.addData("looptime (ms)", loopTime.milliseconds())
//        runAllTelemetries()
    }

    fun runAllTelemetries() {
//        for (subassembly in subassemblyList) subassembly?.telemetry()
//        telemetry.update()
    }
}