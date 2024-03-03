package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class OvercurrentProtection(val motor: DcMotorEx, val activationThreshold: Double, fallbackWarn: () -> Unit, fallbackError: () -> Unit) {

    var isActive = false
    var fallbackStage = 0
    val timer = ElapsedTime()
    val current
        get() = motor.getCurrent(CurrentUnit.AMPS)
    val checker = Thread {
        motor.setCurrentAlert(activationThreshold, CurrentUnit.AMPS)
        while (isActive) {
            fallbackStage = when {
                motor.isOverCurrent && timer.seconds() > 1.0 || current > activationThreshold * 1.5 -> 2 // severe
                motor.isOverCurrent -> 1 // mild
                else -> 0 // none
            }
            when (fallbackStage) {
                2 -> fallbackError() // severe
                1 -> fallbackWarn() // mild
                0 -> { // none
                    timer.reset()
                    if (!motor.isMotorEnabled) motor.setMotorEnable()
                }
            }
        }
    }

    fun start() {
        isActive = true
        checker.start()
    }
    fun stop() { isActive = false }
}