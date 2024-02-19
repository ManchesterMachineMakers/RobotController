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
import org.firstinspires.ftc.teamcode.util.powerCurve

@TeleOp(name = "Semi-Automatic TeleOp", group = "arm")
class SemiAutoTeleOp: LinearOpMode() {

    init { telemetry.isAutoClear = false } // ensure subassembly ready messages aren't cleared

    // control
    var relativeWristAlignment = Arm.WristAlignment.EASEL

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



    // control variables
    val register = mutableSetOf<String>()
    override fun runOpMode() {
        // init, no movement allowed
        subassemblyStatuses = "init"

        runAllTelemetries()
        telemetry.update()

        waitForStart()

        if (opModeIsActive()) {
            // start
            subassemblyStatuses = "start"
            arm.overcurrentProtection.start()
            telemetry.isAutoClear = true
            while (opModeIsActive()) {
                subassemblyStatuses = "loop"

                // Subassembly control
                driveBase.control(gamepad1)
                pixelReleaseControl(gamepad2)
                winchControl(gamepad2.right_stick_y)
                droneLauncherControl(gamepad2.x)
                armControl(gamepad2)

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

    fun armControl(gamepad: Gamepad) {
        val armMotor = arm.armMotor

        if (!armMotor.isOverCurrent) // lock out controls if overcurrent
            armMotor.power = powerCurve(gamepad.left_stick_y.toDouble())

        armMotor.mode = // arm calibration
            if (gamepad.b) DcMotor.RunMode.STOP_AND_RESET_ENCODER
            else DcMotor.RunMode.RUN_USING_ENCODER

        if (gamepad.a) relativeWristAlignment = Arm.WristAlignment.FLOOR
        if (gamepad.y) relativeWristAlignment = Arm.WristAlignment.EASEL

        arm.wrist.position = arm.relativeWristPosition(armMotor.currentPosition, relativeWristAlignment)
    }
}