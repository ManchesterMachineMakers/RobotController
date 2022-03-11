package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.StaticData;

@Autonomous(name = "Data Write Test", group = "Diagnostics")
@Disabled
public class DataTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotLog.i("Data test" + " " + "Value: " + StaticData.thing);
        waitForStart();
        StaticData.thing = "Yes";
        RobotLog.i("Data test" + " " + "Value: " + StaticData.thing);
    }


}
