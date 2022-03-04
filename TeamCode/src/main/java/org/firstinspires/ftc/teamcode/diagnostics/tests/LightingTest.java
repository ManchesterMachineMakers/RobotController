package org.firstinspires.ftc.teamcode.diagnostics.tests;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FreightFrenzyAutonomous;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;

@Test("Lighting Test")
@Requires(Blinkin.class)
public class LightingTest implements Base {

    Blinkin ledUtil;
    Telemetry.Item ledPattern;
    Telemetry.Item modeLine;

    public void init(Runner runner, Blinkin ledUtil) {
        modeLine = runner.opMode.telemetry.addLine("Current Mode").addData("Running in", "");
        ledPattern = runner.opMode.telemetry.addLine("Lights").addData("LED Pattern", "");
        runner.log("*** Lighting Test Initialized. ***");
    }

    @Override
    public boolean run(Testable[] sel, Runner runner) {
        try {
            ledUtil = Testable.getOrDie(sel, Blinkin.class);
            init(runner, ledUtil);

            // every 3 seconds, change mode.
            // alerts should be shown for 0.5 seconds and then
            // return to the previous pattern.


            modeLine.setValue("Autonomous");

            // *** AUTONOMOUS DEFAULT ***
            ledUtil.autonomousDefault();
            ledPattern.setValue("Autonomous Default");
            runner.log("*** Running Autonomous Lighting Tests. ***");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            // *** RINGS DETECTED ***
            runner.log("*** Lighting Test: Ring Detection Alerts ***");
            ledUtil.detected(FreightFrenzyAutonomous.MuffinPosition.Left);
            ledPattern.setValue("Detect 0 Rings (Alert)");
            runner.log("*** Detect 0 Rings (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.detected(FreightFrenzyAutonomous.MuffinPosition.Middle);
            ledPattern.setValue("Detect 1 Ring (Alert)");
            runner.log("*** Detect 1 Ring (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.detected(FreightFrenzyAutonomous.MuffinPosition.Right);
            ledPattern.setValue("Detect 4 Rings (Alert)");
            runner.log("*** Detect 4 Rings (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            if (!letItRun(runner.opMode)) { return abortTest(); }


            // *** LINE DETECTION ***
            runner.log("*** Lighting Test: Line Detection Alerts ***");
            ledUtil.detectedLine(Color.RED);
            ledPattern.setValue("Detect RED Line (Alert)");
            runner.log("*** Detect RED Line (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.detectedLine(Color.WHITE);
            ledPattern.setValue("Detect WHITE Line (Alert)");
            runner.log("*** Detect WHITE Line (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.detectedLine(Color.BLUE);
            ledPattern.setValue("Detect BLUE Line (Alert)");
            runner.log("*** Detect BLUE Line (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            // should show the previous default
            ledUtil.detectedLine(Color.GRAY);
            ledPattern.setValue("Detect GRAY Color (Alert)");
            runner.log("*** Detect GRAY Color (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            // *** WOBBLE GOAL GRABBER ***
            runner.log("*** Lighting Test: Wobble Goal Alert ***");
            ledUtil.grabbedWobbleGoal();
            ledPattern.setValue("Grabbed Wobble Goal (Alert)");
            runner.log("*** Grabbed Wobble Goal (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            // *** RING LOADING ***
            runner.log("*** Lighting Test: Ring Loading Alerts ***");
            ledUtil.needMoreRings();
            ledPattern.setValue("Need More Rings (Alert)");
            runner.log("*** Need More Rings (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.needMoreRings();
            ledPattern.setValue("Loaded One Ring (Alert)");
            runner.log("*** Loaded One Ring (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.ringMagazineFull();
            ledPattern.setValue("Ring Magazine Full (Alert)");
            runner.log("*** Ring Magazine Full (Alert)");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            // *** SHOOTING ***
            runner.log("*** Lighting Test: Shooting ***");
            ledUtil.outOfRange();
            ledPattern.setValue("Out Of Range");
            runner.log("*** Out Of Range");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.almostReadyToShoot();
            ledPattern.setValue("Almost Ready To Shoot");
            runner.log("*** Almost Ready To Shoot");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.readyToShoot();
            ledPattern.setValue("Ready To Shoot");
            runner.log("*** Ready To Shoot");

            if (!letItRun(runner.opMode)) { return abortTest(); }


            // *** TELEOP MODE FEEDBACK ***
            modeLine.setValue("TeleOp");

            ledUtil.teleOpDefault();
            ledPattern.setValue("TeleOp Default");
            runner.log("*** TeleOp");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            // *** SHOOTING ***
            runner.log("*** Lighting Test: Shooting ***");
            ledUtil.outOfRange();
            ledPattern.setValue("Out Of Range");
            runner.log("*** Out Of Range");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.almostReadyToShoot();
            ledPattern.setValue("Almost Ready To Shoot");
            runner.log("*** Almost Ready To Shoot");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.readyToShoot();
            ledPattern.setValue("Ready To Shoot");
            runner.log("*** Ready To Shoot");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.autonomousAction();
            ledPattern.setValue("Autonomous Action During TeleOp");
            runner.log("*** Auto-TeleOp");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            // *** GAME PERIODS ***
            /*
            runner.log("*** Lighting Test: Game Period Defaults ***");
            ledUtil.autonomousDefault();
            ledPattern.setValue("Autonomous Default");
            runner.log("*** Autonomous");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.teleOpDefault();
            ledPattern.setValue("TeleOp Default");
            runner.log("*** TeleOp");

            if (!letItRun(runner.opMode)) { return abortTest(); }
            */

            ledUtil.endgameDefault();
            ledPattern.setValue("Endgame Default");
            runner.log("*** EndGame");

            if (!letItRun(runner.opMode)) { return abortTest(); }

            ledUtil.gameOver();
            ledPattern.setValue("Game Over");
            runner.log("*** Game Over");
            if (!letItRun(runner.opMode)) { return abortTest(); }
            runner.log("*** Lighting Test: Completed Successfully. ***");
            return true;

        } catch (Exception ex) {
            runner.log("*** ABORTED! ***");
            runner.log(ex.getMessage());
            RobotLog.logStackTrace(ex);
            return abortTest();
        }

    }

    private boolean letItRun(LinearOpMode opMode) {
        boolean ok = true;
        opMode.telemetry.update();
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            ok = false;
        }
        return ok;
    }

    private boolean abortTest() {
        RobotLog.i("16221 Diagnostics System: LightingTest *** Lighting Test: Aborted ***");
        return false;
    }
}
