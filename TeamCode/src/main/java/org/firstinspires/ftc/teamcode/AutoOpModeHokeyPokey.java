package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;

@Autonomous(name = "The Hokey Pokey", group = "Freight Frenzy")
public class AutoOpModeHokeyPokey extends MMMFreightFrenzyOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // You put your Whole Self in
        oneMeasure(DriveBase.TravelDirection.forward, 4);
        // You put your Whole Self out
        oneMeasure(DriveBase.TravelDirection.reverse, 4);
        // You put your Whole Self in
        oneMeasure(DriveBase.TravelDirection.forward, 4);
        // And you shake it all about
        intake.go(DcMotorSimple.Direction.FORWARD);
        oneMeasure(DriveBase.TravelDirection.strafeLeftForward, 1);
        oneMeasure(DriveBase.TravelDirection.strafeRightBackward, 1);
        intake.stop();
        intake.go(DcMotorSimple.Direction.REVERSE);
        oneMeasure(DriveBase.TravelDirection.strafeRightForward, 1);
        oneMeasure(DriveBase.TravelDirection.strafeLeftBackward, 1);
        intake.stop();
        // You do the Hokey Pokey and you turn yourself around
        delivery.runSlideToPosition(1);
        oneMeasure(DriveBase.TravelDirection.pivotLeft, 4);
        delivery.runSlideToPosition(2);
        oneMeasure(DriveBase.TravelDirection.pivotRight, 4);
        // That's what it's all about!
        intake.go(DcMotorSimple.Direction.FORWARD);
        delivery.runSlideToPosition(3);
        oneMeasure(DriveBase.TravelDirection.reverse, 4);
        delivery.runSlideToPosition(0);
        intake.stop();
    }

    private void oneMeasure(DriveBase.TravelDirection whichWayDoWeGo, int beats) throws InterruptedException {
        driveBase.go(
                whichWayDoWeGo,
                DriveBase.DriveSpeed.FAST
                );
        keepTheBeat(beats);
        driveBase.stop();
    }

    private void keepTheBeat(int beats) throws InterruptedException {
        while (opModeIsActive() && driveBase.isBusy() && beats < 0) {
            wait(1000); // wait one second
            beats--;
        }
    }
}
