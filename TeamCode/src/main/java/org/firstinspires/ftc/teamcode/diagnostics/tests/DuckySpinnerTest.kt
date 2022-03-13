package org.firstinspires.ftc.teamcode.diagnostics.tests

import org.firstinspires.ftc.teamcode.diagnostics.Runner
import org.firstinspires.ftc.teamcode.diagnostics.util.KTestable
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.subassemblies.DuckySpinner
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad
import java.lang.Exception

@Test("Ducky Spinner Test")
@Requires(DuckySpinner::class)
class DuckySpinnerTest : Base {
    override fun run(sel: Array<out Testable>?, runner: Runner): Boolean {
        val duckySpinner = KTestable.getOrDie<DuckySpinner>(sel)

        runner.log("Running.")
        duckySpinner.start(0.5)
        runner.opMode.sleep(500)
        runner.log("Stopping.")
        duckySpinner.stop()

        try {
            val gmp1 = KTestable.getOrDie<Gamepad>(sel)[1]
            while(!gmp1.ps) {
                duckySpinner.controller()
            }
        } catch (e: Exception) {
            runner.log("No gamepad to be found.")
        }

        runner.log("Done.")
        return true
    }
}