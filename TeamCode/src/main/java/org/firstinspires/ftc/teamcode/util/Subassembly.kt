package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.*

abstract class Subassembly(protected val opMode: OpMode, protected val gamepad: Gamepad) {
    abstract val name: String

    protected val telemetry: Telemetry = opMode.telemetry
    protected val hardwareMap: HardwareMap = opMode.hardwareMap
    protected val loopTime = ElapsedTime()

    val gamepadManager
        get() = GamepadManager(gamepad)

    var status = "unknown"

    open fun telemetry() {
        telemetry.addData(name, status)
        telemetry.addData("runtime (s)", floor(opMode.runtime))
        telemetry.addData("loop time (ms)", floor(loopTime.milliseconds()))
        telemetry.addLine()
    }

    abstract fun loop()
}