package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;

@Autonomous(name = "The Hokey Pokey", group = "Freight Frenzy")
public class AutoOpModeHokeyPokey extends MMMFreightFrenzyOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException if opMode is terminated
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        // Start it talking
        report("Are you ready?");

        waitForStart();
        runtime.reset();
        
        // You put your Whole Self in
        report("You put your Whole Self in,");
        hokeyPokey(DriveBase.TravelDirection.forward, 4);
        // You put your Whole Self out
        report("You put your Whole Self out,");
        hokeyPokey(DriveBase.TravelDirection.reverse, 4);
        // You put your Whole Self in
        report("You put your Whole Self in,");
        hokeyPokey(DriveBase.TravelDirection.forward, 4);
        // And you shake it all about
        report("and you shake it all about!");
        intake.go(DcMotorSimple.Direction.FORWARD);
        hokeyPokey(DriveBase.TravelDirection.strafeLeftForward, 1);
        hokeyPokey(DriveBase.TravelDirection.strafeRightBackward, 1);
        intake.stop();
        intake.go(DcMotorSimple.Direction.REVERSE);
        hokeyPokey(DriveBase.TravelDirection.strafeRightForward, 1);
        hokeyPokey(DriveBase.TravelDirection.strafeLeftBackward, 1);
        intake.stop();
        // You do the Hokey Pokey and you turn yourself around
        report("You do the Hokey Pokey,");
        if (delivery != null) {
            delivery.runSlideToPosition(1);
        }
        hokeyPokey(DriveBase.TravelDirection.pivotLeft, 4);
        report(" and you turn yourself around,");
        if (delivery != null) {
            delivery.runSlideToPosition(2);
        }
        hokeyPokey(DriveBase.TravelDirection.pivotRight, 4);
        // That's what it's all about!
        report("That's what it's all about!");
        intake.go(DcMotorSimple.Direction.FORWARD);
        if (delivery != null) {
            delivery.runSlideToPosition(3);
        }
        hokeyPokey(DriveBase.TravelDirection.reverse, 4);
        if (delivery != null) {
            delivery.runSlideToPosition(0);
        }
        intake.stop();
    }

    private void report(String message) {
        telemetry.speak(message);
        telemetry.addLine(message);
        telemetry.update();
    }

    private void hokeyPokey(DriveBase.TravelDirection whichWayDoWeGo, int beats) throws InterruptedException {
        driveBase.go(
                whichWayDoWeGo,
                DriveBase.DriveSpeed.FAST
                );
        keepTheBeat(beats);
        driveBase.stop();
    }

    private void keepTheBeat(int beats) throws InterruptedException {
        while (opModeIsActive() && driveBase.isBusy() && beats > 0) {

            Thread.sleep(900); // wait one second
            beats--;
        }
    }
}
