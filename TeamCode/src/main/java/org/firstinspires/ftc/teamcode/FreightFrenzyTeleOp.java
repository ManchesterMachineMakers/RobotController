package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;

@TeleOp(name = "Freight Frenzy Full", group = "Freight Frenzy")
public class FreightFrenzyTeleOp extends MMMFreightFrenzyOpMode {
    double r;
    double robotAngle;
    double rightX;
    double v1;
    double v2;
    double v3;
    double v4;
    double periodLength = 120; //seconds
    double endgameLength = 30; //seconds
    boolean inEndgame = false;
//    int pitchDegrees = 0;
    public void runOpMode() throws InterruptedException {
        initOpMode();

        waitForStart();
        runtime.reset();
        driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // our driving loop goes here
        while (opModeIsActive()) {

//            led.teleOpDefault();

            loopOpMode();

            if ((!inEndgame) && (runtime.seconds() >= (periodLength - endgameLength)))  {
                inEndgame = true;
//                led.endgameDefault();
            }
        }

    }

    /**
     * Handle the driving around from the gamepad.
     */
    public void loopOpMode() {

        driving();

        // default controls for the delivery are defined in the Subassembly
        delivery.controller(this);

        // default controls for the intake are defined in the Subassembly
        intake.controller();

        composeTelemetry();
    }

    public void driving() {
        r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;
        v1 = r * Math.cos(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.sin(robotAngle) + rightX;
        v4 = r * Math.cos(robotAngle) - rightX;

        driveBase.go(new double[] { v1, v2, v3, v4 });
    }

    public void composeTelemetry() {
        telemetry.addLine("Current Slide Position")
                .addData("Motor", delivery.motorPosition);
        telemetry.addLine("Current Chute Servo Positions")
                .addData("Left", delivery.chuteServoLeftPosition )
                .addData("Right", delivery.chuteServoRightPosition );
        telemetry.addLine("Current Door Servo Position").addData( "Door", delivery.doorServoPosition );
        telemetry.addLine("Intake Motor").addData("Power", intake.currentPower);
        telemetry.addLine("Velocity")
                .addData("v1", v1)
                .addData("v2", v2)
                .addData("v3", v3)
                .addData("v4", v4);
        telemetry.update();
    }



}
