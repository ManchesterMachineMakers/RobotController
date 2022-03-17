package org.firstinspires.ftc.teamcode.util.pid;
import org.firstinspires.ftc.teamcode.util.isWithinTolerance;

typealias PIDCorrection = PIDScope.(initial: Double, current: Double, target: Double, correction: Double) -> Double

class PIDScope(val k: Double, val error: Double, val tolerance: Double, val initial: Double, val target: Double) {
    fun calculateCorrection(): Double {
        return k * (error / (target - initial))
    }
}

fun runPID(initial: Double, target: Double, tolerance: Double, k: Double, correction: PIDCorrection): Double {
    var current = initial
    while(!isWithinTolerance(current, target, tolerance)) {
        current = correction(PIDScope(k, target - current, tolerance, initial, target), initial, current, target, target - current)
    }
    return current
}