package org.firstinspires.ftc.teamcode.diagnostics.tests

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.diagnostics.util.KTestable
import org.firstinspires.ftc.teamcode.diagnostics.Runner
import org.firstinspires.ftc.teamcode.util.pid.runPID
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad

@Test("PID Controller Test")
@Requires(Gamepad::class)
class PIDTest : Base {
    override fun run(sel: Array<Testable>?, runner: Runner): Boolean {
        var k = 100.0
        val initial = 0.0
        val target = 3600.0
        val tolerance = 0.01

        val gamepad = KTestable.getOrDie<Gamepad>(sel).get(1)
        
        val telemetry = runner.opMode.telemetry
        var correctionLn = telemetry.addData("Correction", 0)
        var initialLn = telemetry.addData("Initial", initial)
        var currentLn = telemetry.addData("Current", initial)
        var targetLn = telemetry.addData("Target", target)
        var toleranceLn = telemetry.addData("Tolerance", tolerance)
        var errorLn = telemetry.addData("Error", 0)
        var kLn = telemetry.addData("Constant", k)

        fun actualRun() {
            runner.log("Correction\tInitial\tCurrent\tTarget\tTolerance\tError\tConstant")
            runner.log("0\t$initial\t$initial\t$target\t$tolerance\t0\t$k")
            val final = runPID(initial, target, tolerance, k) { initialInLoop, current, targetInLoop, correction ->
                val actCorrection = calculateCorrection()
                runner.log("$actCorrection\t$initialInLoop\t$current\t$targetInLoop\t$tolerance\t$correction\t$k")
                correctionLn.setValue(actCorrection)
                currentLn.setValue(current)
                errorLn.setValue(correction)
                kLn.setValue(k)
                runner.opMode.telemetry.update()
                current + actCorrection
            }
            runner.log("Finished PID loop; final: $final, target: $target")
        }

        telemetry.addLine("Initializing PID test; use dpad up/down to adjust constant, X to run test")
        runner.log("Initializing PID test; use dpad up/down to adjust constant, X to run test")
        while(runner.opMode.opModeIsActive() && !gamepad.ps) {
            if(gamepad.dpad_up) {
                k++
                kLn.setValue(k)
            } else if(gamepad.dpad_down) {
                k--
                kLn.setValue(k)
            }
            if(gamepad.x) {
                actualRun()
            }
            telemetry.update()
        }

        return true
    }
}