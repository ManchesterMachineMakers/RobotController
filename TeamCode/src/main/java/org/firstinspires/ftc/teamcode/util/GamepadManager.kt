package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad

class GamepadManager(val gamepad: Gamepad) {
    var wasPressed = mutableSetOf<String>()
    private fun getButton(name: String) = Gamepad::class.java.getDeclaredField(name).getBoolean(gamepad)
    fun on(button: String, closure: () -> Unit) {
        if(getButton(button)) {
            wasPressed.add(button)
            closure()
        }
    }
    fun off(buttons: Collection<String>, closure: () -> Unit) {
        if(wasPressed.any { buttons.contains(it) } && !buttons.any { getButton(it) }) {
            wasPressed.removeAll(buttons)
            closure()
        }
    }
    fun once(button: String, closure: () -> Unit) {
        if(wasPressed.contains(button)) {
            if(!getButton(button)) wasPressed.remove(button)
            return
        }
        if(getButton(button)) {
            wasPressed.add(button)
            closure()
        }
    }
}