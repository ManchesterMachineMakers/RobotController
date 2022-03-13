package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly

class DuckySpinner(private val opMode: LinearOpMode) : Subassembly {
    val wheel = opMode.hardwareMap.crservo[KtHardware.name("servo_DuckWheel")]
    var running = false
    fun start(power: Double) {
        wheel.power = power
        running = true
    }
    fun stop() {
        wheel.power = 0.0
        running = false
    }
    fun controller() {
        val gamepad = when(KtHardware.name("duckWheel_Gamepad")) {
            "gamepad1" -> opMode.gamepad1
            else -> opMode.gamepad2
        }
        if(gamepad.right_trigger > 0.1) {
            if(!running) start(0.5)
        } else if(gamepad.left_trigger > 0.1) {
            if(!running) start(-0.5)
        } else {
            stop()
        }
    }
}