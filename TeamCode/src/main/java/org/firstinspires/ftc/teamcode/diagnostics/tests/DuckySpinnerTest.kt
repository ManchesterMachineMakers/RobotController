package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.subassemblies.DuckySpinner
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad

fun duckySpinnerTest(opMode: DiagnosticsOpMode) = describe<DuckySpinner> { duckySpinner ->
    log("Running.")
    duckySpinner.start(0.5)
    opMode.sleep(500)
    log("Stopping.")
    duckySpinner.stop()

    try {
        val gmp1 = require<Gamepad>()[1]
        while(!gmp1.ps) {
            duckySpinner.controller()
        }
    } catch (_: Throwable) {
        log("No gamepad to be found.")
    }
}