package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly

class DuckySpinner(opMode: LinearOpMode) : Subassembly {
    private val wheel = opMode.hardwareMap.crservo[KtHardware.name("servo_DuckWheel")]
    fun start() {
        wheel.power = 0.5
    }
    fun stop() {
        wheel.power = 0.0
    }
}