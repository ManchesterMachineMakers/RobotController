package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.GamepadTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

@TeleOp(name = "Gamepad Diagnostic", group = "Diagnostics")
public class Diagnostics_GamepadOnly extends DiagnosticsOpMode {
    @Override
    protected void tests() {
        test(GamepadTest.class);
    }

    @Override
    public Testable[] provides() {
        try {
            return new Testable[] {
                    RobotConfig.CURRENT.getHardware(Gamepad.class, this)
            };
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }
        return new Testable[] {
                new Gamepad(this)
        };
    }
}
