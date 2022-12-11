package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion

abstract class CustomBlocksOpModeCompanion : BlocksOpModeCompanion(), Subject {
    abstract fun exists(): Boolean
    companion object {
        fun <T : CustomBlocksOpModeCompanion> setHardwareMap(thing: Class<T>, theHardwareMap: HardwareMap, gamepad1: Gamepad, gamepad2: Gamepad) {
            thing.getField("hardwareMap").set(null, theHardwareMap)
            thing.getField("gamepad1").set(null, gamepad1)
            thing.getField("gamepad2").set(null, gamepad2)

            try {
                thing.getMethod("initHardware").invoke(null);
            } catch (_: Throwable) {
                // no initialization required
            }
        }
    }
}