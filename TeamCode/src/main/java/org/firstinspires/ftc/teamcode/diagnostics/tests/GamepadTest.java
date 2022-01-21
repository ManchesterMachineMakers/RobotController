package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;

@Test("Gamepad Test")
@Requires(Gamepad.class)
public class GamepadTest implements Base {

    @Override
    public boolean run(Testable[] sel, Runner runner) throws Exception {
        com.qualcomm.robotcore.hardware.Gamepad gmp1 = Testable.getOrDie(sel, Gamepad.class).get(1);
        Telemetry.Item lx = runner.opMode.telemetry.addData("Left Stick X", gmp1.left_stick_x);
        Telemetry.Item ly = runner.opMode.telemetry.addData("Left Stick Y", gmp1.left_stick_y);
        Telemetry.Item rx = runner.opMode.telemetry.addData("Right Stick X", gmp1.right_stick_x);
        Telemetry.Item ry = runner.opMode.telemetry.addData("Right Stick Y", gmp1.right_stick_y);
        while(!runner.opMode.gamepad1.left_bumper) {
            lx.setValue(gmp1.left_stick_x);
            ly.setValue(gmp1.left_stick_y);
            rx.setValue(gmp1.right_stick_x);
            ry.setValue(gmp1.right_stick_y);
            //TODO: This fails with the exception "Cannot have more than 255 string data points."
            runner.opMode.telemetry.update();
        }
        return true;
    }
}
