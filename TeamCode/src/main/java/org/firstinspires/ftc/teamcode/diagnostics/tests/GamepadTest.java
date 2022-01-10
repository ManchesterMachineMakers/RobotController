package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;

@Test("Gamepad Test")
public class GamepadTest implements Base {
    @Override
    public Class<? extends Testable>[] requires() {
        return new Class[0];
    }

    @Override
    public boolean run(Testable[] sel, Runner runner) {
        while(!runner.opMode.gamepad1.left_bumper) {
            runner.opMode.telemetry.addData("Left Stick X", runner.opMode.gamepad1.left_stick_x);
            runner.opMode.telemetry.addData("Left Stick Y", runner.opMode.gamepad1.left_stick_y);
            runner.opMode.telemetry.addData("Right Stick X", runner.opMode.gamepad1.right_stick_x);
            runner.opMode.telemetry.addData("Right Stick Y", runner.opMode.gamepad1.right_stick_y);
            //TODO: This fails with the exception "Cannot have more than 255 string data points."
            runner.opMode.telemetry.update();
        }
        return true;
    }
}
