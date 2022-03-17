package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.diagnostics.Runner
import org.firstinspires.ftc.teamcode.diagnostics.tests.PIDTest
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "PID Controller Diagnostic")
class Diagnostics_PIDOnly : DiagnosticsOpMode() {
    override fun provides() = arrayOf<Testable>()
    override fun runTests(runner: Runner?) {
        runner?.run(PIDTest())
    }
}