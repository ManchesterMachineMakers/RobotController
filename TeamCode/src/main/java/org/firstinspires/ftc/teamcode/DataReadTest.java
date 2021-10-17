package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.StaticData;
@Autonomous(name = "Data Read Test")
public class DataReadTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        RobotLog.i("Data test" + " " + "Value: " + StaticData.thing);
    }


}
