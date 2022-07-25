package org.manchestermachinemakers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OpMode extends LinearOpMode {
    public void runOpMode() {
        RobotConfig.activate();
        run();
    }
    public abstract void run();
}
