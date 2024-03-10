package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Subject

abstract class Subassembly(protected val opMode: OpMode, protected val name: String): Subject {

    var status = "unknown"
    protected val telemetry = opMode.telemetry
    protected val hardwareMap = opMode.hardwareMap
    protected val runtime = opMode.runtime

    init {
        hardwareMap ?: throw NullPointerException(
            "${opMode::class}.hardwareMap is NULL. Check that your subassembly is initialized in " +
            if (opMode is LinearOpMode) "runOpMode()"
            else "init()"
        )
    }

    open fun telemetry() {
        telemetry.addLine(name)
    }
}