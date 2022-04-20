package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.DeliveryTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

@TeleOp(name = "Delivery Diagnostic", group = "Diagnostics")
public class Diagnostics_DeliveryOnly extends DiagnosticsOpMode {
    @Override
    public Testable[] provides() {
        try {
            return new Testable[] {
                    RobotConfig.CURRENT.getHardware(Delivery.class, this),
                    RobotConfig.CURRENT.getHardware(Gamepad.class, this)
            };
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }
        return new Testable[] {
                new Delivery(this),
                new Gamepad(this)
        };
    }

    @Override
    protected void runTests(Runner runner) throws InterruptedException {
        runner.run(new DeliveryTest());
    }

}