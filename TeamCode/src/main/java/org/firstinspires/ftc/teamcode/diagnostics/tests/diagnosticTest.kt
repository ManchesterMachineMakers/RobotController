package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.Subject
import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode

class FailingDiagnosticTestingThingy : Subject {
    fun returnTrue() = false
}

class DiagnosticTestingThingy : Subject {
    fun returnTrue() = true
}
fun diagnosticTest(opMode: DiagnosticsOpMode) = describe<DiagnosticTestingThingy> { thingy ->
    runner.log("Howdy World! Waiting 1 second")
    opMode.sleep(1000)
    it("returns true after half a second") {
        opMode.sleep(500)
        assert(thingy.returnTrue())
    }
}

fun failingDiagnosticTest(opMode: DiagnosticsOpMode) = describe<FailingDiagnosticTestingThingy> { thingy ->
    runner.log("Howdy World! Waiting 1 second")
    opMode.sleep(1000)
    it("returns true after half a second") {
        opMode.sleep(500)
        assert(thingy.returnTrue())
    }
}