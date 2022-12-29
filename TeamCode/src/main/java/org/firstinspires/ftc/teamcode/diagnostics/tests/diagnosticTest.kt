package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.Subject
import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode

class DiagnosticTestingThingy : Subject {
    fun returnTrue() = true
}
fun diagnosticTest(opMode: DiagnosticsOpMode) = describe<DiagnosticTestingThingy> { thingy ->
    runner.log("Howdy World!")
    it("returns true at the correct time") {
        assert(thingy.returnTrue())
    }
}