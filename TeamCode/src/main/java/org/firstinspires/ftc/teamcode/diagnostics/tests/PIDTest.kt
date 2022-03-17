package org.firstinspires.ftc.teamcode.diagnostics.tests

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.diagnostics.Runner
import org.firstinspires.ftc.teamcode.util.pid.runPID

@Test("PID Controller Test")
class PIDTest : Base {
    override fun run(sel: Array<Testable>?, runner: Runner): Boolean {
        val initial = 0.0
        val target = 6.0
        val tolerance = 0.01
        runner.log("Correction\tInitial\tCurrent\tTarget\tTolerance\tError")
        runner.log("0\t$initial\t$initial\t$target\t$tolerance\t0")
        val final = runPID(initial, target, tolerance, 2.0) { initialInLoop, current, targetInLoop, correction ->
            val actCorrection = calculateCorrection()
            runner.log("$actCorrection\t$initialInLoop\t$current\t$targetInLoop\t$tolerance\t$correction")
            current + actCorrection
        }
        runner.log("Finished PID loop; final: $final, target: $target")
        return true
    }
}