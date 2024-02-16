package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.DroneLauncher
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases
import org.firstinspires.ftc.teamcode.subassemblies.Winch
import org.firstinspires.ftc.teamcode.util.OvercurrentProtection
import org.firstinspires.ftc.teamcode.util.powerCurve

@TeleOp(name = "Semi-Automatic TeleOp", group = "arm")
class SemiAutoTeleOp: LinearOpMode() {

    init { telemetry.isAutoClear = false } // ensure subassembly ready messages aren't cleared

    // subassemblies
    val driveBase = DriveBase(this)
    val arm = Arm(this)
    val pixelReleases = PixelReleases(this)
    val winch = Winch(this)
    val droneLauncher = DroneLauncher(this)

    val subassemblyList = listOf(driveBase, arm, pixelReleases, winch, droneLauncher)

    val overcurrentProtection = OvercurrentProtection(arm.armMotor, 5.0, {
        arm.armMotor.targetPosition = 0
        arm.armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }, {
        requestOpModeStop()
    })

    // control variables
    val register = mutableSetOf<String>()
    override fun runOpMode() {
        // init, no movement allowed

        runAllTelemetries()
        telemetry.update()

        waitForStart()

        if (opModeIsActive()) {
            // start
            overcurrentProtection.start()
            telemetry.isAutoClear = true
            while (opModeIsActive()) {

                // TODO: ARM CODE HERE

                // Subassembly control (except arm)
                driveBase.control(gamepad1)
                pixelReleaseControl(gamepad2)
                winchControl(gamepad2.right_stick_y)
                droneLauncherControl(gamepad2.x)

                runAllTelemetries()
                telemetry.update()
            }
        }
    }

    fun runAllTelemetries() {
        for (subassembly in subassemblyList) subassembly.telemetry()
        telemetry.update()
    }

    fun droneLauncherControl(button: Boolean) { // I need to figure out GamepadManager, but this'll have to do.
        if (button && !register.contains("droneLauncher")) {
            register.add("droneLauncher")
            droneLauncher.launcher.toggle()
        }
        else if (!button) register.remove("droneLauncher")
    }

    fun winchControl(joystick: Float) { winch.winchMotor.power = powerCurve(joystick.toDouble()) }

    fun pixelReleaseControl(gamepad: Gamepad) {
        if (gamepad.left_bumper) pixelReleases.left.open()
        else if (gamepad.left_trigger > 0.05) pixelReleases.left.close()

        if (gamepad.right_bumper) pixelReleases.right.open()
        else if (gamepad.right_trigger > 0.05) pixelReleases.right.close()
    }
}