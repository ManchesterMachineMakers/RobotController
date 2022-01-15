package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;

@TeleOp
public class Diagnostics_ProgrammingBoard extends DiagnosticsOpMode {

    @Override
    public Testable[] provides() {
        return new Testable[] {
                new ProgrammingBoardDriveBase(hardwareMap),
                //new Delivery(hardwareMap),
                //new ActiveIntake(hardwareMap, this)
        };
    }
}
