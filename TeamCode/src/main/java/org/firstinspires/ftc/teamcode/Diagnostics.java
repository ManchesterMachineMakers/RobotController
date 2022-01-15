package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;

@TeleOp
public class Diagnostics extends DiagnosticsOpMode {

    @Override
    public Testable[] provides() {
        return new Testable[] {
            new MecanumDriveBase(hardwareMap),
            new Delivery(hardwareMap),
            new Blinkin(hardwareMap)
        };
    }


}
