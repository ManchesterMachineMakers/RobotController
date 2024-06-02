package org.firstinspires.ftc.teamcode.dashboard;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.Arm;
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.DroneLauncher;
import org.firstinspires.ftc.teamcode.subassemblies.PixelReleases;
import org.firstinspires.ftc.teamcode.subassemblies.Winch;

@Config
@TeleOp(name="Dash Camera", group="Dashboard")
public class DashCamera extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DriveBase driveBase = new DriveBase(this);
        Arm arm = new Arm(this);
        DroneLauncher droneLauncher = new DroneLauncher(this);
        PixelReleases pixelReleases = new PixelReleases(this);
        Winch winch = new Winch(this);

        waitForStart();

        if(opModeIsActive())

            while(opModeIsActive()) {
                driveBase.control(gamepad1);
                arm.control(gamepad2);
                droneLauncher.control(gamepad2.x);
                pixelReleases.control(gamepad2);
                winch.control(gamepad2.right_stick_y);
            }
    }
}
