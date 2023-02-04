package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion

abstract class CustomBlocksOpModeCompanion : BlocksOpModeCompanion(), Subject {
    abstract fun exists(): Boolean
    lateinit var opMode: OpMode

    open fun initHardware() {}
    companion object {
        fun <T : CustomBlocksOpModeCompanion> initProperties(thing: Class<T>, opMode: LinearOpMode) {
            thing.getField("hardwareMap").set(null, opMode.hardwareMap)
            thing.getField("gamepad1").set(null, opMode.gamepad1)
            thing.getField("gamepad2").set(null, opMode.gamepad2)
            thing.getField("opMode").set(null, opMode)
            thing.getField("linearOpMode").set(null, opMode)
            thing.getField("telemetry").set(null, opMode.telemetry)
        }
    }
}