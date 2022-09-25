package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap

object RobotConfig {
    var hwMap: HardwareMap? = null
    @JvmStatic
    fun activate(hwMap: HardwareMap, configuration: (HardwareMap) -> Unit) {
        RobotConfig.hwMap = hwMap
        configuration(hwMap)
    }

    inline fun <reified T> getHardware(name: String) = hwMap?.get(T::class.java, name)
}