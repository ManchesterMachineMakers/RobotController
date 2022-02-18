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
        Telemetry.Item lb = runner.opMode.telemetry.addData("Left Stick Button", gmp1.left_stick_button);
        Telemetry.Item rb = runner.opMode.telemetry.addData("Right Stick Button", gmp1.right_stick_button);

        Telemetry.Item padl = runner.opMode.telemetry.addData("Left Pad", gmp1.dpad_left);
        Telemetry.Item padu = runner.opMode.telemetry.addData("Up Pad", gmp1.dpad_up);
        Telemetry.Item padr = runner.opMode.telemetry.addData("Right Pad", gmp1.dpad_right);
        Telemetry.Item padd = runner.opMode.telemetry.addData("Down Pad", gmp1.dpad_down);

        Telemetry.Item lTrig = runner.opMode.telemetry.addData("Left Trigger", gmp1.left_trigger);
        Telemetry.Item rTrig = runner.opMode.telemetry.addData("Right Trigger", gmp1.right_trigger);

        Telemetry.Item lBump = runner.opMode.telemetry.addData("Left Bumper", gmp1.left_bumper);
        Telemetry.Item rBump = runner.opMode.telemetry.addData("Right Bumper", gmp1.right_bumper);

        Telemetry.Item aButton = runner.opMode.telemetry.addData("A", gmp1.a);
        Telemetry.Item bButton = runner.opMode.telemetry.addData("B", gmp1.b);
        Telemetry.Item xButton = runner.opMode.telemetry.addData("X", gmp1.x);
        Telemetry.Item yButton = runner.opMode.telemetry.addData("Y", gmp1.y);

        Telemetry.Item backButton = runner.opMode.telemetry.addData("Back", gmp1.back);
        Telemetry.Item startButton = runner.opMode.telemetry.addData("Start", gmp1.start);

        while (!runner.opMode.gamepad1.ps && runner.opMode.opModeIsActive()) {
            lx.setValue(gmp1.left_stick_x);
            ly.setValue(gmp1.left_stick_y);
            rx.setValue(gmp1.right_stick_x);
            ry.setValue(gmp1.right_stick_y);
            lb.setValue(gmp1.left_stick_button);
            rb.setValue(gmp1.right_stick_button);

            padl.setValue(gmp1.dpad_left);
            padu.setValue(gmp1.dpad_up);
            padr.setValue(gmp1.dpad_right);
            padd.setValue(gmp1.dpad_down);

            lTrig.setValue(gmp1.left_trigger);
            rTrig.setValue(gmp1.right_trigger);
            lBump.setValue(gmp1.left_bumper);
            rBump.setValue(gmp1.right_bumper);

            aButton.setValue(gmp1.a);
            bButton.setValue(gmp1.b);
            xButton.setValue(gmp1.x);
            yButton.setValue(gmp1.y);

            backButton.setValue(gmp1.back);
            startButton.setValue(gmp1.start);


            runner.opMode.telemetry.update();
        }
        runner.opMode.telemetry.addLine("Ready to Rumble");
        runner.opMode.telemetry.update();
        runner.opMode.gamepad1.rumble(100);
        runner.opMode.telemetry.addLine("Rumble complete.");
        runner.opMode.telemetry.update();
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