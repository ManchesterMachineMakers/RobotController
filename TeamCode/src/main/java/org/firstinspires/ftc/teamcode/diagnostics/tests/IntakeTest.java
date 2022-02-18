package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;

@Test("Intake Test")
@Requires(ActiveIntake.class)
@Requires(Gamepad.class)
public class IntakeTest implements Base {

    ActiveIntake intake;
    Gamepad gamepad;
    Telemetry.Item speedReport;
    Telemetry.Line statusReport;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public boolean run(Testable[] sel, Runner runner) throws Exception {

        intake = Testable.getOrDie(sel, ActiveIntake.class);

        runner.opMode.telemetry.setAutoClear(false);
        statusReport = runner.opMode.telemetry.addLine("Intake Test");
        speedReport = runner.opMode.telemetry.addLine("Active Intake").addData("Speed", intake.getSpeed());
        runner.opMode.telemetry.update();

        intake.initIntakeMotor();
        try {
            intake.go(DcMotorSimple.Direction.FORWARD);
            waitForIt();
            runner.opMode.telemetry.update();

            intake.stop();
            waitForIt();
            runner.opMode.telemetry.update();

            intake.go(DcMotorSimple.Direction.REVERSE);
            waitForIt();
            runner.opMode.telemetry.update();

            intake.stop();
            speedReport.setValue(intake.getSpeed());
            runner.opMode.telemetry.update();

        } catch (InterruptedException ex) {
            statusReport.addData("Exception", ex.getStackTrace());
            runner.opMode.telemetry.update();

        }

        try {
        // if we don't have a gamepad, we won't run the gamepad part.
            gamepad = Testable.getOrDie(sel, Gamepad.class);
            com.qualcomm.robotcore.hardware.Gamepad gmp1 = gamepad.get(1);

            statusReport.addData("Waiting", "for user input.");

            while(runner.opMode.opModeIsActive() && !gmp1.ps) {
                intake.controller();
            }

        } catch (Exception ex) {
            statusReport.addData("Done", "No gamepad to be found. Exiting.");
            runner.opMode.telemetry.update();
        }

        statusReport.addData("Finished", "Running Intake Test.");
        runner.opMode.telemetry.update();
        return true;
    }

    private void waitForIt() throws InterruptedException {
            timer.reset();
            while (timer.milliseconds() < 1000) {
                Thread.sleep(20);
                speedReport.setValue(intake.getSpeed());
            }
    }

}
