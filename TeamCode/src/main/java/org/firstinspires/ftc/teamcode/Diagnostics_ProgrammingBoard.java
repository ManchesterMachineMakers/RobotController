package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.DeliverySelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.DriveBaseSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.IntakeSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.LightingSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

@TeleOp
public class Diagnostics_ProgrammingBoard extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        Blinkin ledUtil = new Blinkin(hardwareMap);
        DriveBase driveBase = new ProgrammingBoardDriveBase(hardwareMap);
        Delivery delivery = new Delivery(hardwareMap);
        ActiveIntake intake = new ActiveIntake(hardwareMap, this);
        Runner runner = new Runner(new Selector[] {
            new DriveBaseSelector(driveBase),
//            new LightingSelector(ledUtil),

            new DeliverySelector(delivery),

            new IntakeSelector(intake)
        }, this);
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