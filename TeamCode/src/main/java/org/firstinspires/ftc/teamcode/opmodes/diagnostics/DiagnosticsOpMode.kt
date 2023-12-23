package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Runner
import com.rutins.aleks.diagonal.Subject
import com.rutins.aleks.diagonal.Test
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.tests.visionTest
import kotlin.reflect.KFunction1

abstract class DiagnosticsOpMode(
        // What tests to run.
        vararg val tests: KFunction1<DiagnosticsOpMode, Pair<Class<out Any>, (Array<Subject>, Runner) -> Test>> = arrayOf(
            ::visionTest
        )
) : LinearOpMode() {
    // Get all the subassemblies currently on the robot.
    open fun provides(): Array<Subject> = arrayOf(Arm::class, DriveBase::class, Vision::class)
            .mapNotNull { it.constructors.find { it.parameters.firstOrNull()?.type == HardwareMap::class }?.call(hardwareMap) }
            .toTypedArray()


    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // create a test runner with all the hardware and a logger
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