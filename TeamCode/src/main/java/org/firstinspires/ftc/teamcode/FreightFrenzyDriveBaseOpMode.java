package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;

@TeleOp(name = "Drive Base Demo")
public class FreightFrenzyDriveBaseOpMode extends MMMFreightFrenzyOpMode {
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
    }
    public void driving() {
        r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;
        v1 = r * Math.cos(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.sin(robotAngle) + rightX;
        v4 = r * Math.cos(robotAngle) - rightX;

        telemetry.addData("v1", v1);
        telemetry.addData("v2", v2);
        telemetry.addData("v3", v3);
        telemetry.addData("v4", v4);
        telemetry.update();

        driveBase.go(new double[] { v1, v2, v3, v4 });
    }
}
