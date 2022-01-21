package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.GamepadTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

public class Diagnostics_GamepadOnly extends DiagnosticsOpMode {
    @Override
    protected void tests() {
        test(GamepadTest.class);
    }
}
