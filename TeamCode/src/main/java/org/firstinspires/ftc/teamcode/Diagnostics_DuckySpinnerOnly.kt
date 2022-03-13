package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.diagnostics.Runner
import org.firstinspires.ftc.teamcode.diagnostics.tests.DuckySpinnerTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.subassemblies.DuckySpinner
import org.firstinspires.ftc.teamcode.util.KtHardware

class Diagnostics_DuckySpinnerOnly : DiagnosticsOpMode() {
    override fun provides() = arrayOf(
        DuckySpinner(this)
    )

    override fun runTests(runner: Runner?) {
        runner?.run(DuckySpinnerTest())
    }
}