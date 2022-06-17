package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.util.pid.runPID
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad

fun pidTest(opMode: DiagnosticsOpMode) = describe<Gamepad> { gamepads ->
    it("can run a PID loop") {
        var k = 100.0
        val initial = 0.0
        val target = 3600.0
        val tolerance = 0.01

        val gamepad = gamepads[1]

        val telemetry = opMode.telemetry
        val correctionLn = telemetry.addData("Correction", 0)
        val initialLn = telemetry.addData("Initial", initial)
        val currentLn = telemetry.addData("Current", initial)
        val targetLn = telemetry.addData("Target", target)
        val toleranceLn = telemetry.addData("Tolerance", tolerance)
        val errorLn = telemetry.addData("Error", 0)
        val kLn = telemetry.addData("Constant", k)

        fun actualRun() {
            log("Correction\tInitial\tCurrent\tTarget\tTolerance\tError\tConstant")
            log("0\t$initial\t$initial\t$target\t$tolerance\t0\t$k")
            val final = runPID(opMode, initial, target, tolerance, k) { initialInLoop, current, targetInLoop, correction ->
                val actCorrection = calculateCorrection()
                log("$actCorrection\t$initialInLoop\t$current\t$targetInLoop\t$tolerance\t$correction\t$k")
                correctionLn.setValue(actCorrection)
                currentLn.setValue(current)
                errorLn.setValue(correction)
                kLn.setValue(k)
                opMode.telemetry.update()
                current + actCorrection
            }
            log("Finished PID loop; final: $final, target: $target")
        }

        telemetry.addLine("Initializing PID test; use dpad up/down to adjust constant, X to run test")
        log("Initializing PID test; use dpad up/down to adjust constant, X to run test")
        while (opMode.opModeIsActive() && !gamepad.ps) {
            if (gamepad.dpad_up) {
                k += 10
                if (target < k) k = target
                kLn.setValue(k)
                opMode.sleep(50)
            } else if (gamepad.dpad_down) {
                k -= 10
                if (initial > k) k = initial
                kLn.setValue(k)
                opMode.sleep(50)
            }
            if (gamepad.x) {
                actualRun()
            }
            telemetry.update()
        }
    }
}