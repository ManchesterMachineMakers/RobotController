package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.diagnostics.tests.DiagnosticTestingThingy
import org.firstinspires.ftc.teamcode.diagnostics.tests.FailingDiagnosticTestingThingy
import org.firstinspires.ftc.teamcode.diagnostics.tests.diagnosticTest
import org.firstinspires.ftc.teamcode.diagnostics.tests.failingDiagnosticTest

@TeleOp(name = "Diagnostic Diagnostic")
class DiagnosticDiagnostic : DiagnosticsOpMode(::diagnosticTest, ::failingDiagnosticTest) {
    override fun provides(): Array<Subject> = arrayOf(DiagnosticTestingThingy(), FailingDiagnosticTestingThingy())
}