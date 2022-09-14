package org.manchestermachinemakers

import com.qualcomm.robotcore.hardware.HardwareMap
import org.manchestermachinemakers.RobotConfig

object RobotConfig {
    var hwMap: HardwareMap? = null
    @JvmStatic
    fun activate(hwMap: HardwareMap) {
        RobotConfig.hwMap = hwMap
    }

    inline fun <reified T> getHardware(name: String) = hwMap?.get(T::class.java, name)
}