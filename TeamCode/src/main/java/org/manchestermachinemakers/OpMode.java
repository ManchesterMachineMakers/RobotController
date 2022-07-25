package org.manchestermachinemakers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.manchestermachinemakers.hardware.drivebase.DriveBase;
import org.manchestermachinemakers.hardware.drivebase.config.ProgrammingBoard;

public abstract class OpMode extends LinearOpMode {
    public void runOpMode() {
        RobotConfig.activate(hardwareMap);
        DriveBase.use(new ProgrammingBoard());
        run();
    }
    public abstract void run();
}
