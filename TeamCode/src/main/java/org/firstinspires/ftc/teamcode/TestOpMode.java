package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;

@TeleOp(name = "Test Op Mode")
public class TestOpMode extends MMMFreightFrenzyOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        waitForStart();

        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBase.go(DriveBase.TravelDirection.forward, 0.7, 500);
        while(driveBase.isBusy() && opModeIsActive());
    }
}
