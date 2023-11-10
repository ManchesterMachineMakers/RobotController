package org.firstinspires.ftc.teamcode.contracts

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.GamepadManager

// Controllable interface
interface Controllable {
    fun controller(gamepad: GamepadManager)

    companion object {
        fun runAll(vararg things: Pair<Gamepad, Controllable>) =
                things.forEach { thing -> thing.second.controller(GamepadManager(thing.first)) }
    }
}
