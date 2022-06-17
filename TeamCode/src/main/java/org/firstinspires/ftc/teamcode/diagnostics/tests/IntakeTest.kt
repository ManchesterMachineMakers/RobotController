package org.firstinspires.ftc.teamcode.diagnostics.tests

import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad
import java.lang.Exception

fun intakeTest(opMode: DiagnosticsOpMode) = describe<ActiveIntake> { intake ->
    opMode.telemetry.isAutoClear = false
    val statusReport = opMode.telemetry.addLine("Intake Test")
    val speedReport = opMode.telemetry.addLine("Active Intake").addData("Speed", intake.speed)
    val timer = ElapsedTime()

    fun waitForIt() {
        timer.reset()
        while (timer.milliseconds() < 1000) {
            Thread.sleep(20)
            speedReport.setValue(intake.speed)
        }
    }

    it("can run") {
        opMode.telemetry.update()
        intake.initIntakeMotor()
        try {
            intake.go(DcMotorSimple.Direction.FORWARD)
            waitForIt()
            opMode.telemetry.update()
            intake.stop()
            waitForIt()
            opMode.telemetry.update()
            intake.go(DcMotorSimple.Direction.REVERSE)
            waitForIt()
            opMode.telemetry.update()
            intake.stop()
            speedReport.setValue(intake.speed)
            opMode.telemetry.update()
        } catch (ex: InterruptedException) {
            statusReport.addData("Exception", ex.stackTrace)
            opMode.telemetry.update()
        }
    }

    it("can be controlled") {
        try {
            // if we don't have a gamepad, we won't run the gamepad part.
            val gamepad = require<Gamepad>()
            val gmp1 = gamepad.get(1)

            statusReport.addData("Waiting", "for user input.")

            val motorPower = opMode.telemetry.addData("Motor Power", intake.motor.power)
            val lTrig = opMode.telemetry.addData("Left Trigger", gmp1.left_trigger)
            val rTrig = opMode.telemetry.addData("Right Trigger", gmp1.right_trigger)
            val lBump = opMode.telemetry.addData("Left Bumper", gmp1.left_bumper)
            val rBump = opMode.telemetry.addData("Right Bumper", gmp1.right_bumper)

            opMode.telemetry.update()

            while (opMode.opModeIsActive() && !gmp1.ps) {
                intake.controller()
                motorPower.setValue(intake.motor.power)
                speedReport.setValue(intake.speed)
                lTrig.setValue(gmp1.left_trigger)
                rTrig.setValue(gmp1.right_trigger)
                lBump.setValue(gmp1.left_bumper)
                rBump.setValue(gmp1.right_bumper)
                opMode.telemetry.update()
            }

        } catch (ex: Exception) {
            statusReport.addData("Done", "No gamepad to be found. Exiting.")
            opMode.telemetry.update()
        }
    }
    statusReport.addData("Finished", "Running Intake Test.")
    opMode.telemetry.update()
}