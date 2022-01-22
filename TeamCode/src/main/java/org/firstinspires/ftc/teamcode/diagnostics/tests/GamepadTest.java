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
        while (!runner.opMode.gamepad1.left_bumper) {
            lx.setValue(gmp1.left_stick_x);
            ly.setValue(gmp1.left_stick_y);
            rx.setValue(gmp1.right_stick_x);
            ry.setValue(gmp1.right_stick_y);
            //TODO: This fails with the exception "Cannot have more than 255 string data points."
            runner.opMode.telemetry.update();
        }
        return true;
    }

    /*** Controls ***
     *
     Match TeleOp
     Gamepad 1 - Drivebase steering/speed control with joysticks
     Gamepad 2 - Intake and Delivery controls
     Intake
     RB - Take in
     RT - Take in (slow)
     LB - Push out
     LT - Push out (slow)
     Delivery
     A - “home” position: chute at 30 degrees, resting on drivebase
     X - level 1 delivery
     Y - level 2 delivery
     B - level 3 delivery
     D-pad up/down: override/precision control of chute height

     Diagnostics mode
     Assign all controls to Gamepad 1
     Add control of chute angle to D-pad left/right
     *
     *** ***/
}