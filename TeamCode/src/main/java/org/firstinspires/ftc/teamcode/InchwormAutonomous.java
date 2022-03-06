//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.navigation.org.firstinspires.ftc.teamcode.util.pathfinder.Destination;
//import org.firstinspires.ftc.teamcode.navigation.FieldDestinations;
//import org.firstinspires.ftc.teamcode.navigation.Location;
//import org.firstinspires.ftc.teamcode.navigation.Movement;
//import org.firstinspires.ftc.teamcode.navigation.MyPath;
//import org.firstinspires.ftc.teamcode.util.MMMUltimateGoalOpMode;
//import org.firstinspires.ftc.teamcode.util.RingDetectorTimeout;
//import org.firstinspires.ftc.teamcode.util.RobotReport;
//
//import java.util.ArrayList;
//
//@Autonomous(name="Inchworm Autonomous")
//public class InchwormAutonomous extends MMMUltimateGoalOpMode {
//
//    org.firstinspires.ftc.teamcode.util.pathfinder.Destination targetZone = FieldDestinations.TZA; // default Target Zone A
//    org.firstinspires.ftc.teamcode.util.pathfinder.Destination startLine = FieldDestinations.SLC; // we assume the left start line, center.
//    int stage = 0;
//
//    private void detectRingStack() {
//        rings = RingDetectorTimeout.detect(this, camera);
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initOpMode();
//
//        // set rings on bot.
//        RobotReport.ringsOnBot = 3;
//        // we also have a wobble goal but we don't need to keep track of that.
//
//        waitForStart();
//
//        // where are we? Left or right start line?
//        Location startLineLocation = reckoning.whereAmI();
//        MyPath startLinePath = Movement.calculatePath(startLineLocation, FieldDestinations.SLC, startLineLocation.orientation);
//        if (reckoning.areWeThereYet(FieldDestinations.SLC, startLinePath, 200, reckoning.angleTolerance)) {
//            // if we're within 20cm, we're on the left start line.  That means we need to go to the left start line tip to see the ring stack.
//            runToDestination(FieldDestinations.SLT);
//        } else {
//            // Otherwise, better change it to the right start line.
//            startLine = FieldDestinations.SRC;
//            runToDestination(FieldDestinations.SRT);
//        }
//        // face the ring stack so that we can see it with the camera.
//        changeFacing(FieldDestinations.STACK);
//
//        // we look for the ring stack.
//        detectRingStack();
//        robotReport.itemRunReport.setValue("Ring Stack Detected");
//        robotReport.update();
//        robotReport.logReport();
//
//        // we could now calculate which is closer - the shooting position or the target area, so we
//        // know whether to go deliver the goals first, or whether we should shoot first and ask questions later.
//        //TODO: Calculations
//        // If we start on the Left start line, it's probably more efficient to shoot first.
//        // If we start on the Right start line, it may be more efficient to deliver the first wobble goal,
//        // then shoot and then deliver the second.
//
//        // go shoot the rings we have on the robot.
//        // if there are rings in the starter stack, we will go and load more rings onto the bot and shoot those, too.
//        goShootSomeRings();
//        robotReport.itemRunReport.setValue("Rings are all gone, hopefully into the goal!");
//        robotReport.update();
//        robotReport.logReport();
//
//        // based on the number of rings in the starter stack, we will deliver the on-board wobble goal to the target zone.
//        // then we will go and get the other wobble goal. We know where that is because it's on the other start line, and
//        // we already know which one we started on.
//        goDeliverSomeGoals();
//        robotReport.itemRunReport.setValue("Goals delivered to the target zone.");
//        robotReport.update();
//        robotReport.logReport();
//
//        //** that's it, we're done - go park on the launch line.
//        if (runToDestination(FieldDestinations.LLP)) {
//            // celebrate!
//        } else {
//            // bummer!
//        }
//        robotReport.itemRunReport.setValue("Finished!");
//        robotReport.update();
//        robotReport.logReport();
//
//    }
//
//    /**
//     * Head to the shooting location, shoot the three preloaded rings.
//     * Then check the starter stack for more rings, take them in, and shoot them.
//     * @return success
//     */
//    private boolean goShootSomeRings() {
//        //TODO: Add telemetry and logging.
//        //TODO: Add pitch calculations for  inchworm  drivebase.
//
//        boolean success;
//        // We begin with a wobble goal and 3 rings loaded. Where first?
//
//        //** go shoot some preloaded rings.
//        // drive to it.
//        success = runToDestination(FieldDestinations.SHOOT);
//        if (!success) {
//            return false;
//        }
//
//        // shoot all the rings we have on the robot.
//        aimAndShoot(FieldDestinations.TOW, RobotReport.ringsOnBot);
//
//        //** go get the starter stack.  We already determined how many there are.
//        //** now go there and intake up to 3 rings.
//        if (rings.ringCount > 0) {
//            success = runToDestination(FieldDestinations.STACK);
//            if (!success) {
//                return false;
//            }
//
//            // The RingSensor should tell us when we take on rings.
//            // The intake should stop as soon as it detects the third ring.
//            // this could leave a ring in the intake; when we shoot we should
//            // start the intake motor again to be sure.
//            //TODO: We need something to tell the intake how long to run!
//            // Try a color sensor.
//            // Not getting a color sensor.  Use timing instead. Run the intake for ...roughly 2 seconds, how about?
//            while (RobotReport.ringsOnBot <= 3 && RobotReport.ringsOnBot <= rings.ringCount && opModeIsActive()) {
//                intake.go(DcMotorSimple.Direction.FORWARD);
//                this.sleep(700);
//                RobotReport.ringsOnBot++;
//                // idle();
//            }
//            intake.stop();
//
//            //* go shoot what we took in.
//            if (RobotReport.ringsOnBot > 0) {
//                // go shoot them
//                // drive to it.
//                success = runToDestination(FieldDestinations.SHOOT);
//                if (!success) {
//                    return false;
//                }
//
//                success = aimAndShoot(FieldDestinations.PSL, 1);
//                if (!success) {
//                    return false;
//                }
//                // we shot one.  try for the others if we have them.
//                if (aimAndShoot(FieldDestinations.PSC, 1)) {
//                    aimAndShoot(FieldDestinations.PSR, 1);
//                }
//            }
//
//            //* were there more rings? Go fetch the last one.
//            if (rings.ringCount == 4) {
//                success = runToDestination(FieldDestinations.STACK);
//                if (!success) {
//                    return false;
//                }
//
//                // The RingSensor should tell us when we take on rings.
//                // Here, there will only be one ring left, so we tell it to stop after 1.
//                while (RobotReport.ringsOnBot <= 1 && opModeIsActive()) {
//                    intake.go(DcMotorSimple.Direction.FORWARD);
//                    idle();
//                }
//                intake.stop();
//
//                //* go shoot what we took in.
//                if (RobotReport.ringsOnBot > 0) {
//                    success = runToDestination(FieldDestinations.SHOOT);
//                    // drive to it.
//                    if (!success) {
//                        return false;
//                    }
//
//                    success = aimAndShoot(FieldDestinations.TOW, 1);
//                    if (!success) {
//                        return false;
//                    }
//                }
//            }
//        }
//        return true;
//    }
//
//    /**
//     * Shoot up to 3 shots, aimed at a particular goal.
//     * @param goal Which goal are we shooting at?
//     * @param howManyShots How many rings should we shoot?
//     * @return success
//     */
//    private boolean aimAndShoot(org.firstinspires.ftc.teamcode.util.pathfinder.Destination goal, int howManyShots) {
//        //TODO: does Isaac have a routine that calculates firing position on the field for a given shooter angle?
//        try {
//            if (RobotReport.ringsOnBot < 1 || (!opModeIsActive())) {
//                led.needMoreRings();
//                return false;
//            }
//            boolean success;
//            success = changeFacing(goal);
//            //TODO: change pitch if needed for shooting.
//
//            if (success) {
//                // spin up the flywheel
//                shooter.prepForShooting();
//                // make sure all rings are in the magazine
//                intake.go(DcMotorSimple.Direction.FORWARD);
//                // go for it.
//                for (int i = Math.min(howManyShots, RobotReport.ringsOnBot); (i > 0 && opModeIsActive()); i--) {
//                    // shoot 3 preloaded rings.
//                    while (!shooter.isUpToSpeed() && opModeIsActive()) {
//                        robotReport.itemFlywheelSpeed.setValue("Shooter spinning up . . .");
//                        robotReport.update();
//                        idle();
//                    }
//                    robotReport.itemFlywheelSpeed.setValue("Flywheel is up to speed!");
//                    robotReport.update();
//                    led.readyToShoot();
//
//                    if (opModeIsActive()) {
//                        shooter.shoot();
//                        // the shooter decrements the ring count when it shoots.
//                        robotReport.itemRingsOnBot.setValue(RobotReport.ringsOnBot);
//                    }
//                    robotReport.update();
//                }
//            }
//            return success;
//        } catch (Exception ex) {
//            return false;
//        }
//    }
//
//    /**
//     * Deliver one wobble goal to the target zone, then go fetch the other and deliver it too.
//     * @return success
//     */
//    private boolean goDeliverSomeGoals() {
//        //TODO: Add telemetry and logging.
//
//        ArrayList<org.firstinspires.ftc.teamcode.util.pathfinder.Destination> course = new ArrayList<>();
//        boolean success;
//        // We begin with a wobble goal loaded.
//        // We detected our Target Zone.
//        org.firstinspires.ftc.teamcode.util.pathfinder.Destination targetZone = rings.targetZone;
//        if (targetZone == null) {
//            // our backup is A, because that's for 0 rings.
//            // we're least likely to get a good reading if it's 0.
//            targetZone = FieldDestinations.TZA;
//        }
//        //** go to the target zone.
//        course.add(targetZone);
//        //TODO: will we need to adjust position to drop the goal?
//        success = runCourse(course);
//        if (!success) { return false; }
//        //** drop the loaded wobble goal in the target zone.
//        //TODO: fine-tune the wobbleGoalGrabber methods.
//        wobbleGoalGrabber.releaseWrapper();
//        course.clear();
//
//        //** go get the other one.  It will be on the other start line, tip.
//        if (startLine == FieldDestinations.SLC) {
//            course.add(FieldDestinations.SRT);
//        } else {
//            course.add(FieldDestinations.SLT);
//        }
//        //TODO: will we need to adjust position to pick up the goal?
//        success = runCourse(course);
//        if (!success) { return false; }
//        //** drop the loaded wobble goal in the target zone.
//        //TODO: fine-tune the wobbleGoalGrabber methods.
//        wobbleGoalGrabber.grabWrapper();
//        course.clear();
//
//        //** go to the target zone to drop the second one.
//        course.add(targetZone);
//        //TODO: will we need to adjust position to drop the goal?
//        // both will need to be fully in the target zone.
//        success = runCourse(course);
//        if (!success) { return false; }
//        //** drop the loaded wobble goal in the target zone.
//        //TODO: fine-tune the wobbleGoalGrabber methods.
//        wobbleGoalGrabber.releaseWrapper();
//
//        return true;
//    }
//
//}
