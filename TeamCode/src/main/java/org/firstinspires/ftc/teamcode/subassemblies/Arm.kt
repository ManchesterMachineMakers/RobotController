package org.firstinspires.ftc.teamcode.subassemblies

import org.manchestermachinemakers.easyop.Subassembly
import org.manchestermachinemakers.easyop.Device
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.*

class Arm : Subassembly {
    @Device("motorEXP0") lateinit var armMotor: DcMotor
    @Device("servo0") lateinit var parallel: Servo
    @Device("servo1") lateinit var rightDropper: Servo
    @Device("servo2") lateinit var leftDropper: Servo

    data class PlacementInfo(val distToBase: Double, val alpha: Double, val beta: Double)

    companion object {
        // Constants
        val gamma = atan2(16., 283.)
        val l2 = 67.88
        val l3 = 59.56
        val r = 336.0
        val h = 287.75
        val d1 = 220.0
        val d2 = 88.9
    }

    fun getPlacementInfo(pixelRow: Int): PlacementInfo {
        val beta = asin(((((d1 + pixelRow*d2) - l3) * sin(PI / 3) + l2 * cos(PI / 3)) - h) / r)
        val alpha = PI/2 + PI/3 - gamma - beta
        val distToBase = r * cos(beta) + l2 * sin(PI/3) + cos(PI/3) * ((l3 - d1) - pixelRow * d2)
        return PlacementInfo(distToBase, alpha, beta)
    }
}