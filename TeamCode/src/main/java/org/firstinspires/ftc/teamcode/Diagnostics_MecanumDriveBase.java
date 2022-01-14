package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.DeliverySelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.DriveBaseSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.IntakeSelector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

@TeleOp
public class Diagnostics_MecanumDriveBase extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveBase driveBase = new MecanumDriveBase(hardwareMap);
        Runner runner = new Runner(new Selectors() {
            @Override
            public Selector<DriveBase> driveBaseSelector() {
                return new DriveBaseSelector(driveBase);
            }

            @Override
            public Selector<Blinkin> lightingSelector() {
                return null;
            }

            @Override
            public Selector<Delivery> deliverySelector() {
                return null;
            }

            @Override
            public Selector<ActiveIntake> activeIntakeSelector() {
                return null;
            }

            @Override
            public Selector<Vision> cameraSelector() {
                return null;
            }
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