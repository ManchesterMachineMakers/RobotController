package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;

@Autonomous(name = "Hokey Pokey", group = "Freight Frenzy")
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
        driveBase.go(
                DriveBase.TravelDirection.forward,
                DriveBase.DriveSpeed.FAST
                );
        // You put your Whole Self out
        // You put your Whole Self in
        // And you shake it all about
        // You do the Hokey Pokey and you turn yourself around
        // That's what it's all about!
    }
}
