package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.LightingTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.LightingSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.WobbleGoalGrabber;

@TeleOp
public class Diagnostics_LightingOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Blinkin ledUtil = new Blinkin(hardwareMap);
        Runner runner = new Runner(new Selectors() {
            @Override
            public Selector<DriveBase> driveBaseSelector() {
                return null;
            }

            @Override
            public Selector<Blinkin> lightingSelector() {
                return new LightingSelector(ledUtil);
            }

            @Override
            public Selector<WobbleGoalGrabber> wggSelector() {
                return null;
            }

            @Override
            public Selector<ActiveIntake> activeIntakeSelector() {
                return null;
            }
        }, this);
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