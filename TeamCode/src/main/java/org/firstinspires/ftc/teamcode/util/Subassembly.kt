package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.*

abstract class Subassembly(protected val opMode: OpMode, protected val gamepad: Gamepad) {
    abstract val name: String

    protected val telemetry = opMode.telemetry!!
    protected val hardwareMap = opMode.hardwareMap!!
    protected val loopTime = ElapsedTime()

    val gamepadManager
        get() = GamepadManager(gamepad)

    var status = ""

    open fun telemetry() {
        telemetry.addLine()
        telemetry.addData(name, status)
        telemetry.addData("runtime (s)", floor(opMode.runtime))
        telemetry.addData("loop time (ms)", floor(loopTime.milliseconds()))
    }

    open fun loop() {
        loopTime.reset()
    }
}