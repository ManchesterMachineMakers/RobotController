package org.firstinspires.ftc.teamcode.diagnostics.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Runner
import com.rutins.aleks.diagonal.Subject
import com.rutins.aleks.diagonal.Test
import com.rutins.aleks.diagonal.TestConstructor
import org.firstinspires.ftc.teamcode.diagnostics.tests.*
import org.firstinspires.ftc.teamcode.util.RobotConfig
import java.lang.Exception
import java.util.ArrayList
import kotlin.reflect.KFunction1

abstract class DiagnosticsOpMode(
        vararg val tests: KFunction1<DiagnosticsOpMode, Pair<Class<out Any>, (Array<Subject>, Runner) -> Test>> = arrayOf(
            ::driveBaseTest,
            ::lightingTest,
            ::deliveryTest,
            ::intakeTest,
            ::duckyVisionTest,
            ::pidTest,
            ::deliveryControllerTest,
            //::gamepadTest,
        )
) : LinearOpMode() {
    open fun provides(): Array<Subject> {
        return RobotConfig.CURRENT.getTestable(this)
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val runner = Runner(tests.map { it(this) }.toTypedArray(), RobotLogger())
        RobotLog.i("16221 Diagnostics Opmode: Initialization complete.")
        telemetry.addLine("Initialized.")
        telemetry.update()
        waitForStart()
        if (opModeIsActive()) {
            RobotLog.i("16221 Diagnostics Opmode: Running all tests.")
            telemetry.addLine("Running tests. Please watch the log.")
            telemetry.update()
            runTests(runner)
        }
    }

    @Throws(InterruptedException::class)
    protected open fun runTests(runner: Runner) {
        runner.runAll(provides())
    }
}