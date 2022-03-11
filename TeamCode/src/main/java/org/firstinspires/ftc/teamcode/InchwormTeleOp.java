package org.firstinspires.ftc.teamcode; //set your package

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.navigation.Location;
import org.firstinspires.ftc.teamcode.util.MMMUltimateGoalOpMode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import static org.firstinspires.ftc.teamcode.navigation.FieldDestinations.TOW;

@TeleOp(name = "InchwormTeleOp", group = "Ultimate Goal")
@Disabled
public class InchwormTeleOp extends MMMUltimateGoalOpMode {

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
    int pitchDegrees = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        waitForStart();
        runtime.reset();
        driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // our driving loop goes here
        while (opModeIsActive()) {

            led.teleOpDefault();

            loopOpMode();

            if ((!inEndgame) && (runtime.seconds() >= (periodLength - endgameLength)))  {
                inEndgame = true;
                led.endgameDefault();
            }
        }

    }

    /**
     * Handle the driving around from the gamepad.
     */
    public void loopOpMode() {

        manualDriving();

        manualPitching();

        manualShooting();

        manualRingIntake();

        autoPrepShooter();

//        manualWGGControl();

        //reckonAndReport();

    }

    private void manualPitching() {
        if(gamepad1.right_trigger > 0.5) {
            pitchDegrees++;
            if(pitchDegrees > 10) pitchDegrees = 10;
            driveBase.pitch(pitchDegrees);
        } if(gamepad1.left_trigger > 0.5) {
            pitchDegrees = 0;
            driveBase.pitch(0);
        }
    }

    /**
     * From Simple2_RD
     * Set up controls for manual driving of the bot.
     */
    private void manualDriving() {

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

    /**
     * Set up controls for manual shooting
     */
    private void manualShooting() {

        if(gamepad1.left_bumper) {
            shooter.stop();
        }
        if(gamepad1.x) {
            shooter.shoot();
        }

    }

    /**
     * Set up controls for manual ring intake
     */
    private void manualRingIntake() {
        if(gamepad1.a) {
            intake.go(Direction.FORWARD);
        }
        if(gamepad1.b) {
            intake.stop();
        }
        if(gamepad1.y) {
            intake.go(Direction.REVERSE);
        }
    }

    /**
     * Set up controls for wobble goal grabber
     */
//    private void manualWGGControl() {
//        if(gamepad1.dpad_down) {
//            wobbleGoalGrabber.down();
//        }
//        if(gamepad1.dpad_up) {
//            wobbleGoalGrabber.up();
//        }
//        if(gamepad1.dpad_left) {
//            wobbleGoalGrabber.grab();
//        }
//        if(gamepad1.dpad_right) {
//            wobbleGoalGrabber.release();
//        }
//    }

    /**
     * From TeleArrayAS - auto aim and shoot. Does not include pitch.
     * Assumes that the driver will place the bot at the appropriate Y distance from the goal,
     * but the bot will then orient itself to face the goal from its X position.
     */
    private void autoPrepShooter() {
        if (gamepad1.right_bumper) {
            String s = Double.toString(stage);
            RobotLog.i("***16221_LOG***   AIM stage start: ", s);

            // spin up the flywheel.
            shooter.prepForShooting();

            led.autonomousAction();
            boolean success;
//            // Rotate the robot to aim at the TOWER goal.
//            Location currentLocation = reckoning.whereAmI();
//
//            // assume a set Y location; the driver will have to get the bot there.
//            currentLocation.setY(-762);
//
//            success = changeFacing(currentLocation, TOW);
            if (inEndgame) {
                led.endgameDefault();
            } else {
                led.teleOpDefault();
            }
        }
    }

    /**
     * determine position and update the robot status.
     * Uses multithreading to prevent latency.
     */
    public void reckonAndReport() {
        Thread tr = new Thread(new Runnable() {
            @Override
            public void run() {
                robotReport.isShooterUpToSpeed = shooter.isUpToSpeed();
                robotReport.intakeSpeed = intake.getSpeed();
                robotReport.runReport = "TELEOP CONTROL";

                // figure out where we are.
                Location fieldPosition = reckoning.whereAmI();
                robotReport.fieldPosition = fieldPosition;
                robotReport.fieldOrientation = fieldPosition.orientation;

                // every loop, update telemetry and lighting status
                robotReport.updateBotStatus();
                robotReport.update();

                // every two seconds, log a report
                if (Math.floorMod((long) runtime.seconds(), 2) == 0) {
                    robotReport.logReport();
                }
            }
        });
        tr.start();
    }
}