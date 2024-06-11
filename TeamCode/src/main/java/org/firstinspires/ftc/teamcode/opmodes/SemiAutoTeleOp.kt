package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.DroneLauncher
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.subassemblies.Winch
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.ShowTelemetry
import org.firstinspires.ftc.teamcode.util.log

@TeleOp(name = "Semi-Automatic TeleOp (preferred)", group = "arm")
class SemiAutoTeleOp: LinearOpMode(), DashOpMode {

    override fun runOpMode() {
        // init, no movement allowed
        telemetry = MultipleTelemetry(DashOpMode.Static.telemetry, telemetry)
//        telemetry.isAutoClear = false

        val driveBase = DriveBase(this)
        val arm = Arm(this)
        val pixelReleases = PixelReleases(this)
        val winch = Winch(this)
        val droneLauncher = DroneLauncher(this)
        val vision = Vision(this)

        vision.visionPortal

        val loopTime = ElapsedTime()
        val subassemblyList = listOf(driveBase, arm, pixelReleases, winch, droneLauncher, vision)
        driveBase.opInit()

        waitForStart()

        if (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)
            log("Starting OpMode loop")
//            telemetry.isAutoClear = true
            telemetry.clear()
            while (opModeIsActive()) {
                loopTime.reset()

                // Subassembly control
                driveBase.control(gamepad1)
                pixelReleases.control(gamepad2)
                winch.control(gamepad2.right_stick_y)
                droneLauncher.control(gamepad2.x)
                arm.control(gamepad2)

                if (ShowTelemetry.JOYSTICK) joystickTelemetry(this)
                if (ShowTelemetry.ARM) arm.telemetry()
                if (ShowTelemetry.DRIVE_BASE) driveBase.telemetry()
                if (ShowTelemetry.DRONE_LAUNCHER) droneLauncher.telemetry()
                if (ShowTelemetry.PIXEL_RELEASES) pixelReleases.telemetry()
                if (ShowTelemetry.WINCH) winch.telemetry()
                telemetry.addData("Loop Time", loopTime.time())
                telemetry.update()
            }
        }
    }
}