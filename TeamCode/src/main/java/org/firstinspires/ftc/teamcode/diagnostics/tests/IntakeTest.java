package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;

public class IntakeTest implements Base {

    ActiveIntake intake;
    Telemetry.Item speedReport;
    Telemetry.Line statusReport;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public boolean run(Selectors sel, Runner runner) {

        intake = sel.activeIntakeSelector().get();

        runner.opMode.telemetry.setAutoClear(false);
        statusReport = runner.opMode.telemetry.addLine("Finished running Intake Test");
        speedReport = runner.opMode.telemetry.addLine("Active Intake").addData("Speed", intake.getSpeed());

        try {
            intake.go(DcMotorSimple.Direction.FORWARD);
            waitForIt();

            intake.stop();
            waitForIt();

            intake.go(DcMotorSimple.Direction.REVERSE);
            waitForIt();

            intake.stop();
        } catch (InterruptedException ex) {
            statusReport.addData("Exception", ex.getStackTrace());
        }
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
