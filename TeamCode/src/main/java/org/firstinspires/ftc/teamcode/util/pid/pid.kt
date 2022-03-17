package org.firstinspires.ftc.teamcode.util.pid;
import org.firstinspires.ftc.teamcode.util.isWithinTolerance;

typealias PIDCorrection = (initial: Double, current: Double, target: Double, correction: Double) -> Double


fun runPID(initial: Double, target: Double, tolerance: Double, correction: PIDCorrection): Double {
    var current = initial
    while(!isWithinTolerance(current, target, tolerance)) {
        current = correction(initial, current, target, target - current)
    }
    return current
}