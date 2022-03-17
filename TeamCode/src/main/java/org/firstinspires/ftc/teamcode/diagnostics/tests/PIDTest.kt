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
        runner.log("At start; initial value: $initial, target: $target, tolerance: $tolerance")
        runPID(initial, target, tolerance) { initialInLoop, current, targetInLoop, correction ->
            runner.log("PID loop; initial value: $initialInLoop, current: $current, target: $targetInLoop, correction: $correction")
            val actCorrection = correction / (target - initial)
            runner.log("PID loop; correcting by $actCorrection")
            current + actCorrection
        }
        return true
    }
}