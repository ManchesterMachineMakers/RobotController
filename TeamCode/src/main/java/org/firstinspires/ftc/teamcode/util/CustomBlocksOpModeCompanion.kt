package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion

abstract class CustomBlocksOpModeCompanion : BlocksOpModeCompanion() {
    companion object {
        fun <T : CustomBlocksOpModeCompanion> setHardwareMap(thing: Class<T>, theHardwareMap: HardwareMap) {
            thing.getField("hardwareMap").set(null, theHardwareMap)

            try {
                thing.getMethod("init").invoke(null);
            } catch (_: NoSuchMethodException) {
                // no initialization required
            }
        }
    }
}