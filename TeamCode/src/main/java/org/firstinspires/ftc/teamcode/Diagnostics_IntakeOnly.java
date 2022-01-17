package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

@TeleOp
public class Diagnostics_IntakeOnly extends DiagnosticsOpMode {

    @Override
    public Testable[] provides() {
        try {
            return new Testable[] {
                    RobotConfig.CURRENT.getHardware(ActiveIntake.class, this)
            };
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }
        return new Testable[] {
                new ActiveIntake(this)
        };
    }
}