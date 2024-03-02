package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Subassembly(protected val opMode: OpMode, protected val name: String) {

    var status = "unknown"
    protected val telemetry = opMode.telemetry
    protected val hardwareMap = opMode.hardwareMap
    protected val runtime = opMode.runtime

    open fun telemetry() {
        telemetry.addLine(name)
        /*
        Pixel Releases:
        isOpen : true, false
        Wrist:
        position: target: 1, actual: 0.98
         */
    }
}