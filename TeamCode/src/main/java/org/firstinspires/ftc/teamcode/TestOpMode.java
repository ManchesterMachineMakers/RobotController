package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name = "Test Op Mode")
public class TestOpMode extends MMMFreightFrenzyOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        waitForStart();

        driveBase.go(DriveBase.TravelDirection.forward, DriveBase.DriveSpeed.FAST);
    }
}
