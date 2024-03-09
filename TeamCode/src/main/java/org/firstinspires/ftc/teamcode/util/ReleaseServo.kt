package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Servo

class ReleaseServo(val servo: Servo, val scaleRange: Pair<Double, Double>, val direction: Servo.Direction = Servo.Direction.FORWARD) {

    var isOpen = servo.position > 0.5
        private set

    init {
        servo.scaleRange(scaleRange.first, scaleRange.second)
        servo.direction = direction
    }

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