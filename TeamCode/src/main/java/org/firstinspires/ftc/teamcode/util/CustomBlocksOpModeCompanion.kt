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
        fun <T : CustomBlocksOpModeCompanion> initProperties(thing: T, opMode: LinearOpMode) {
            thing::class.java.getField("gamepad1")    .set(thing, opMode.gamepad1)
            thing::class.java.getField("hardwareMap") .set(thing, opMode.hardwareMap)
            thing::class.java.getField("gamepad2")    .set(thing, opMode.gamepad2)
            thing::class.java.getField("opMode")      .set(thing, opMode)
            thing::class.java.getField("linearOpMode").set(thing, opMode)
            thing::class.java.getField("telemetry")   .set(thing, opMode.telemetry)
        }
    }
}