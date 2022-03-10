package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;

@TeleOp(name = "Vision Diagnostic", group = "Diagnostics")
public class Diagnostics_Vision extends DiagnosticsOpMode {

    @Override
    public Testable[] provides() {
        return new Testable[] {
            new Vision(this)
        };
    }

}