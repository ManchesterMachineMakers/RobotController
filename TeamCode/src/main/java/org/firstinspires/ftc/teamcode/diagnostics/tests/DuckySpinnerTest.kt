package org.firstinspires.ftc.teamcode.diagnostics.tests

import org.firstinspires.ftc.teamcode.diagnostics.Runner
import org.firstinspires.ftc.teamcode.diagnostics.util.KTestable
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.subassemblies.DuckySpinner

@Test("Ducky Spinner Test")
@Requires(DuckySpinner::class)
class DuckySpinnerTest : Base {
    override fun run(sel: Array<out Testable>?, runner: Runner): Boolean {
        val duckySpinner = KTestable.getOrDie<DuckySpinner>(sel)

        runner.log("Running.")
        duckySpinner.start()
        runner.opMode.sleep(500)
        runner.log("Stopping.")
        duckySpinner.stop()

        return true
    }
}