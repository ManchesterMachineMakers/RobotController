package org.firstinspires.ftc.teamcode.diagnostics.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;

public abstract class DiagnosticsOpMode extends LinearOpMode {
    public abstract Testable[] provides();
    @Override
    public void runOpMode() throws InterruptedException {
        //Blinkin ledUtil = new Blinkin(hardwareMap);
        Runner runner = new Runner(this);

        RobotLog.i("16221 Diagnostics Opmode: Initialization complete.");
        telemetry.addLine("Initialized.");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            RobotLog.i("16221 Diagnostics Opmode: Running all tests.");
            telemetry.addLine("Running tests. Please watch the log.");
            telemetry.update();
            runner.runAll();
        }
    }
}
