package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Servo

class ReleaseServo(val servo: Servo) {

    var isOpen = servo.position > 0.5
    fun open() {
        servo.position = 1.0
        isOpen = true
    }
    fun close() {
        servo.position = 0.0
        isOpen = false
    }
    fun toggle() {
        if (isOpen) close()
        else open()
        isOpen = !isOpen
    }
}