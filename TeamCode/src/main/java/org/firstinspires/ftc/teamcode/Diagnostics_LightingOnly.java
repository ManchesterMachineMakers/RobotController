package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.LightingTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.LightingSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;

@TeleOp
public class Diagnostics_LightingOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Blinkin ledUtil = new Blinkin(hardwareMap);
        Runner runner = new Runner(new Selector[] {new LightingSelector(ledUtil)}, this);
        RobotLog.i("16221 Diagnostics Opmode: Initialization complete.");
        telemetry.addLine("Initialized.");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            RobotLog.i("16221 Diagnostics Opmode: Running lighting tests.");
            telemetry.addLine("Running tests. Please watch the log.");
            telemetry.update();
            runner.run(new LightingTest());
        }
    }


}