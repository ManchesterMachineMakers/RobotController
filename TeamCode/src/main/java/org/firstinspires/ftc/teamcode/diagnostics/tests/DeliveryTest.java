package org.firstinspires.ftc.teamcode.diagnostics.tests;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;

@Test("Delivery Test")
@Requires(Delivery.class)
@Requires(Gamepad.class)
public class DeliveryTest implements Base {

    public boolean run(Testable[] sel, Runner runner) throws Exception {
        Delivery delivery = Testable.getOrDie(sel, Delivery.class);
        runner.log("Home");
        delivery.runSlideToPosition(Delivery.SLIDE_HOME_POSITION);
        runner.log("Low");
        delivery.runSlideToPosition(Delivery.SLIDE_LOW_POSITION);
        Thread.sleep(1000);
        runner.log("Mid");
        delivery.runSlideToPosition(Delivery.SLIDE_MID_POSITION);
        Thread.sleep(1000);
        runner.log("High");
        delivery.runSlideToPosition(Delivery.SLIDE_HIGH_POSITION);
        Thread.sleep(1000);
        runner.log("Cap");
        delivery.runSlideToPosition(Delivery.SLIDE_CAP_POSITION);
        Thread.sleep(1000);
        runner.log("Open");
        delivery.setDoorOpenPosition();
        Thread.sleep(1000);
        runner.log("Closed");
        delivery.setDoorClosedPosition();
        Thread.sleep(1000);
        runner.log("Unfold");
        delivery.setChuteOpenPosition();
        Thread.sleep(1000);
        runner.log("Fold");
        delivery.setChuteCompactPosition();
        Thread.sleep(1000);
        runner.log("Home");
        delivery.runSlideToPosition(0);
        Thread.sleep(1000);
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

            delivery.controller();

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
