package org.firstinspires.ftc.teamcode.util.pid;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.isWithinTolerance;

typealias PIDCorrection = PIDScope.(initial: Double, current: Double, target: Double, correction: Double) -> Double

class PIDScope(val k: Double, val error: Double, val tolerance: Double, val initial: Double, val target: Double) {
    fun calculateCorrection(): Double {
        if (k != 0.0 && (target-initial) != 0.0) {
            return k * (error / (target - initial))
        }
        return target
    }
}

fun runPID(opMode: LinearOpMode, initial: Double, target: Double, tolerance: Double, k: Double, correction: PIDCorrection): Double {
    var current = initial
    while(!isWithinTolerance(current, target, tolerance) && opMode.opModeIsActive() && !opMode.isStopRequested()) {
        current = correction(PIDScope(k, target - current, tolerance, initial, target), initial, current, target, target - current)
    }
    return current
}