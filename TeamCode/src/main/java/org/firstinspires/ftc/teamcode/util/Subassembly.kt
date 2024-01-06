package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.*

abstract class Subassembly(protected val opMode: OpMode, protected val gamepad: Gamepad, protected val name: String) {

    var status = "unknown"
    protected val telemetry: Telemetry = opMode.telemetry
    protected val hardwareMap: HardwareMap = opMode.hardwareMap
    protected val loopTime = ElapsedTime()
    // Extension property (see: https://stackoverflow.com/questions/36502413/extension-fields-in-kotlin)
    private val externalMap = mutableMapOf<DcMotor, Int>()
    private var DcMotor.latestPosition: Int
        get() = externalMap[this] ?: 0
        set(value) { externalMap[this] = value }

    val gamepadManager
        get() = GamepadManager(gamepad)

    open fun telemetry() {
        telemetry.addLine(name)
        telemetry.addData("status", status)
        telemetry.addData("runtime (s)", floor(opMode.runtime))
        telemetry.addData("loop time (ms)", floor(loopTime.milliseconds()))
    }

    /** Make sure this starts with "loopTime.reset()" */
    abstract fun loop()

    protected fun DcMotor.updateLatestPosition() { latestPosition = currentPosition }

    /** Brakes a motor with more power than regular DcMotor.ZeroPowerBehavior.BRAKE */
    protected fun DcMotor.brake(power: Double) {
        targetPosition = latestPosition
        mode = DcMotor.RunMode.RUN_TO_POSITION
        this.power = power
    }
}