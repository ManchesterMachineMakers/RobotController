package org.firstinspires.ftc.teamcode.contracts

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.util.GamepadManager

// Controllable interface
interface Controllable {
    fun controller(gamepad: GamepadManager) 

    companion object {
        fun runAll(vararg things: Pair<Gamepad, Controllable>)
            = things.forEach { thing -> thing.1.controller(thing.0) }
    }
}