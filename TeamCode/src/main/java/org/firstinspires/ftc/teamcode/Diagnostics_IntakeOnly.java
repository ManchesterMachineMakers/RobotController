package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.IntakeTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.IntakeSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.WobbleGoalGrabber;

@TeleOp
public class Diagnostics_IntakeOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //TODO: Add the rest of the intake functionality to this test.
        ActiveIntake intake = new ActiveIntake(hardwareMap, this);

        Runner runner = new Runner(new Selectors() {
            @Override
            public Selector<DriveBase> driveBaseSelector() {
                return null;
            }

            @Override
            public Selector<Blinkin> lightingSelector() {
                return null;
            }

            @Override
            public Selector<WobbleGoalGrabber> wggSelector() {
                return null;
            }

            @Override
            public Selector<ActiveIntake> activeIntakeSelector() {
                return new IntakeSelector(intake);
            }
        }, this);
        RobotLog.i("16221 Diagnostics Opmode: Initialization complete.");
        telemetry.addLine("Initialized.");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            RobotLog.i("16221 Diagnostics Opmode: Running intake tests.");
            telemetry.addLine("Running tests. Please watch the log.");
            telemetry.update();
            //runner.run(new RingSensorTest());
            runner.run(new IntakeTest());
        }
    }


}