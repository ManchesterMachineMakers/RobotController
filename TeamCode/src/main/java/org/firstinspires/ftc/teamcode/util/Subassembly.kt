package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Subassembly(protected val opMode: OpMode, protected val name: String) {

    var status = "unknown"
    protected val telemetry: Telemetry = opMode.telemetry
    protected val hardwareMap: HardwareMap = opMode.hardwareMap
    protected val loopTime = ElapsedTime()

    open fun telemetry() {
        telemetry.addLine(name)
        telemetry.addData("status", status)
    }
}