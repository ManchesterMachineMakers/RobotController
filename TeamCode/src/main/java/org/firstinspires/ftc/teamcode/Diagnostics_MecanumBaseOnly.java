package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.DriveBaseSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;

@TeleOp
public class Diagnostics_MecanumBaseOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveBase driveBase = new MecanumDriveBase(hardwareMap);
        Runner runner = new Runner(new Selector[] {
                new DriveBaseSelector(driveBase)
        }, this);
        RobotLog.i("16221 Diagnostics Opmode: Initialization complete.");
        telemetry.addLine("Initialized.");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            RobotLog.i("16221 Diagnostics Opmode: Running Tests for Mecanum Drive Base Only.");
            telemetry.addLine("Running tests for Mecanum Drive Base Only. Please watch the log.");
            telemetry.update();
            runner.runAll();
        }
    }


}