package org.firstinspires.ftc.teamcode.diagnostics.tests;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;

@Test("Delivery Controller Test")
@Requires(Delivery.class)
@Requires(Gamepad.class)
public class DeliveryControllerTest implements Base {

    public boolean run(Testable[] sel, Runner runner) throws Exception {
        runner.opMode.telemetry.speak("Important! Please retract the slides completely to zero before running this op mode!");
        Thread.sleep(3000);
        runner.opMode.telemetry.speak("Repeat: Please retract the slides completely to zero before running this op mode!");
        Thread.sleep(3000);
        runner.opMode.telemetry.addLine("Slides are now AT ZERO.  If they are not FULLY RETRACTED, you will break them!  If they are not retracted, stop this OpMode, retract the slides, and restart.");
        runner.opMode.telemetry.update();
        Thread.sleep(3000);
        
        Delivery delivery = Testable.getOrDie(sel, Delivery.class);

        runner.log("Testing Controls");

        Telemetry.Item motorPos = runner.opMode.telemetry.addData("Slide Motor Position", delivery.motor.getCurrentPosition());
        Telemetry.Item servoLeftPos = runner.opMode.telemetry.addData("Left Servo Position", delivery.chuteServoLeft.getPosition());
        Telemetry.Item servoRightPos = runner.opMode.telemetry.addData("Right Servo Position", delivery.chuteServoRight.getPosition());
        Telemetry.Item servoDoorPos = runner.opMode.telemetry.addData("Door Servo Position", delivery.doorServo.getPosition());

        com.qualcomm.robotcore.hardware.Gamepad gmp1 = Testable.getOrDie(sel, Gamepad.class).get(1);
        Telemetry.Item padl = runner.opMode.telemetry.addData("Left Pad", gmp1.dpad_left);
        Telemetry.Item padu = runner.opMode.telemetry.addData("Up Pad", gmp1.dpad_up);
        Telemetry.Item padr = runner.opMode.telemetry.addData("Right Pad", gmp1.dpad_right);
        Telemetry.Item padd = runner.opMode.telemetry.addData("Down Pad", gmp1.dpad_down);

        Telemetry.Item aButton = runner.opMode.telemetry.addData("A", gmp1.a);
        Telemetry.Item bButton = runner.opMode.telemetry.addData("B", gmp1.b);
        Telemetry.Item xButton = runner.opMode.telemetry.addData("X", gmp1.x);
        Telemetry.Item yButton = runner.opMode.telemetry.addData("Y", gmp1.y);

        Telemetry.Item backButton = runner.opMode.telemetry.addData("Back", gmp1.back);
        runner.opMode.telemetry.update();

        while (!runner.opMode.gamepad1.ps && runner.opMode.opModeIsActive()) {

            delivery.controller(runner.opMode);

            motorPos.setValue(delivery.motor.getCurrentPosition());
            servoLeftPos.setValue(delivery.chuteServoLeft.getPosition());
            servoRightPos.setValue(delivery.chuteServoRight.getPosition());
            servoDoorPos.setValue(delivery.doorServo.getPosition());

            padl.setValue(gmp1.dpad_left);
            padu.setValue(gmp1.dpad_up);
            padr.setValue(gmp1.dpad_right);
            padd.setValue(gmp1.dpad_down);

            aButton.setValue(gmp1.a);
            bButton.setValue(gmp1.b);
            xButton.setValue(gmp1.x);
            yButton.setValue(gmp1.y);

            backButton.setValue(gmp1.back);
            runner.opMode.telemetry.update();
        }
        runner.log("Done");
        return true;
    }
}
