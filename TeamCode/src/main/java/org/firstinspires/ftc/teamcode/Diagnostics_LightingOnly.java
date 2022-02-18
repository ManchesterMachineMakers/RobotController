package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.LightingTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;

@TeleOp(name = "Lighting Diagnostic", group = "Diagnostics")
public class Diagnostics_LightingOnly extends DiagnosticsOpMode {

    @Override
    public Testable[] provides() {
        return new Testable[] {
            new Blinkin(this)
        };
    }

}