package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;

@Test("Drive Base")
public class DriveBaseTest implements Base {
    @Override
    public boolean run(Selector[] sel, Runner runner) throws Exception {
        DriveBase driveBase = Selector.getOrDie(sel, DriveBase.class).get();
        double power = 0.2;

        runner.log("Testing each motor");
        driveBase.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor[] motors = driveBase.getMotors();
        for (DcMotor motor: motors
             ) {
            runner.log("Motor: " + motor.getDeviceName());
            motor.setPower(power);
            runner.opMode.sleep(500);
            motor.setPower(0-power);
            runner.opMode.sleep(500);
            motor.setPower(0);
        }

        runner.log("Testing coordinated motion");

        runner.log("Moving forward 500 ticks w/ encoder");
//        driveBase.setTravelDirection(DriveBase.TravelDirection.forward);
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        driveBase.setTargetPositions(500);
//        driveBase.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
        driveBase.go(DriveBase.TravelDirection.forward, power,500, 50);
        while(driveBase.isBusy());


        runner.log("Moving backward 500 ticks w/ encoder");
        //driveBase.setTravelDirection(DriveBase.TravelDirection.reverse);
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveBase.setTargetPositions(500);
        //driveBase.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
	driveBase.go(DriveBase.TravelDirection.reverse, power,500, 50);
        while(driveBase.isBusy());

        runner.log("Strafing left 500 ticks w/ encoder");
        //driveBase.setTravelDirection(DriveBase.TravelDirection.strafeLeft);
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveBase.setTargetPositions(500);
        //driveBase.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
	driveBase.go(DriveBase.TravelDirection.strafeLeft, power,500, 50);
        while(driveBase.isBusy());

        runner.log("Strafing right 500 ticks w/ encoder");
        //driveBase.setTravelDirection(DriveBase.TravelDirection.strafeRight);
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveBase.setTargetPositions(500);
        //driveBase.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
	driveBase.go(DriveBase.TravelDirection.strafeRight, power,500, 50);
        while(driveBase.isBusy());

//        runner.log("Pitching to 30 degrees");
//        driveBase.pitch(30);
//        Thread.sleep(1000);
//        runner.log("Returning to straight");
//        driveBase.pitch(0);
//        Thread.sleep(1000);

        runner.log("Finished Drive Base Test.");
        return true;
    }
}
