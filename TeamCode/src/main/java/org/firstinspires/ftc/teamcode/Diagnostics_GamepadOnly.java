package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.GamepadTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

@TeleOp(name = "Diagnostics (Gamepad Only)")
public class Diagnostics_GamepadOnly extends DiagnosticsOpMode {
    @Override
    protected void tests() {
        test(GamepadTest.class);
    }
}
