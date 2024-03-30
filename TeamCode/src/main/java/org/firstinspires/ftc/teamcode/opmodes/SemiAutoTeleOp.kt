package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.DroneLauncher
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Winch
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.teamcode.util.log

@TeleOp(name = "Semi-Automatic TeleOp (preferred)", group = "arm")
class SemiAutoTeleOp: LinearOpMode() {

    override fun runOpMode() {
        // init, no movement allowed
        telemetry.isAutoClear = false

        val driveBase = DriveBase(this)
        val arm = Arm(this)
        val pixelReleases = PixelReleases(this)
        val winch = Winch(this)
        val droneLauncher = DroneLauncher(this)

        val loopTime = ElapsedTime()
        val subassemblyList = listOf(driveBase, arm, pixelReleases, winch, droneLauncher)
        driveBase.opInit()

        waitForStart()

        if (opModeIsActive()) {
            log("Starting OpMode loop")
            telemetry.isAutoClear = true
            telemetry.clear()
            while (opModeIsActive()) {
                loopTime.reset()
                telemetry.addData("G1 Left X", gamepad1.left_stick_x);
                telemetry.addData("G1 Left Y", gamepad1.left_stick_y);
                telemetry.addData("G1 Right X", gamepad1.right_stick_x);
                telemetry.addData("G1 Right XY", gamepad1.right_stick_y);

                // Subassembly control
                driveBase.control(gamepad1)
                pixelReleases.control(gamepad2)
                winch.control(gamepad2.right_stick_y)
                droneLauncher.control(gamepad2.x)
                arm.control(gamepad2)

                arm.telemetry()
                telemetry.addData("Loop Time", loopTime.time())
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