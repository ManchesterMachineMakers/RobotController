package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.diagnostics.Runner
import org.firstinspires.ftc.teamcode.diagnostics.tests.PIDTest

class Diagnostics_PIDOnly : DiagnosticsOpMode() {
    override fun provides() = arrayOf<Testable>()
    override fun runTests(runner: Runner?) {
        runner?.run(PIDTest())
    }
}