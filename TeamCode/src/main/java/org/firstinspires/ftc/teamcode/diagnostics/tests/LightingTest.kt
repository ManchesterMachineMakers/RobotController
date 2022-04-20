package org.firstinspires.ftc.teamcode.diagnostics.tests

import android.graphics.Color
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin
import org.firstinspires.ftc.teamcode.FreightFrenzyAutonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode


fun lightingTest(opMode: DiagnosticsOpMode) = describe<Blinkin> { ledUtil ->
    fun letItRun(opMode: LinearOpMode): Boolean {
        var ok = true
        opMode.telemetry.update()
        try {
            Thread.sleep(3000)
        } catch (e: InterruptedException) {
            ok = false
        }
        return ok
    }

    val modeLine = opMode.telemetry.addLine("Current Mode").addData("Running in", "")
    val ledPattern = opMode.telemetry.addLine("Lights").addData("LED Pattern", "")
    log("*** Lighting Test Initialized. ***")

    it("can show autonomous patterns") {
        // every 3 seconds, change mode.
        // alerts should be shown for 0.5 seconds and then
        // return to the previous pattern.
        modeLine.setValue("Autonomous")

        // *** AUTONOMOUS DEFAULT ***
        ledUtil.autonomousDefault()
        ledPattern.setValue("Autonomous Default")
        log("*** Running Autonomous Lighting Tests. ***")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show muffin detection alerts") {
        // *** RINGS DETECTED ***
        log("*** Lighting Test: Muffin Detection Alerts ***")
        ledUtil.detected(FreightFrenzyAutonomous.MuffinPosition.Left)
        ledPattern.setValue("Left Muffin (Alert)")
        log("*** Left Muffin (Alert)")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.detected(FreightFrenzyAutonomous.MuffinPosition.Middle)
        ledPattern.setValue("Middle Muffin (Alert)")
        log("*** Middle Muffin (Alert)")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.detected(FreightFrenzyAutonomous.MuffinPosition.Right)
        ledPattern.setValue("Right Muffin (Alert)")
        log("*** Right Muffin (Alert)")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show line detection alerts") {
        // *** LINE DETECTION ***
        log("*** Lighting Test: Line Detection Alerts ***")
        ledUtil.detectedLine(Color.RED)
        ledPattern.setValue("Detect RED Line (Alert)")
        log("*** Detect RED Line (Alert)")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.detectedLine(Color.WHITE)
        ledPattern.setValue("Detect WHITE Line (Alert)")
        log("*** Detect WHITE Line (Alert)")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.detectedLine(Color.BLUE)
        ledPattern.setValue("Detect BLUE Line (Alert)")
        log("*** Detect BLUE Line (Alert)")
        letItRun(opMode).expect("Could not complete run")

        // should show the previous default
        ledUtil.detectedLine(Color.GRAY)
        ledPattern.setValue("Detect GRAY Color (Alert)")
        log("*** Detect GRAY Color (Alert)")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show wobble goal alerts") {
        // *** WOBBLE GOAL GRABBER ***
        log("*** Lighting Test: Wobble Goal Alert ***")
        ledUtil.grabbedWobbleGoal()
        ledPattern.setValue("Grabbed Wobble Goal (Alert)")
        log("*** Grabbed Wobble Goal (Alert)")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show ring loading alerts") {
        // *** RING LOADING ***
        log("*** Lighting Test: Ring Loading Alerts ***")
        ledUtil.needMoreRings()
        ledPattern.setValue("Need More Rings (Alert)")
        log("*** Need More Rings (Alert)")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.needMoreRings()
        ledPattern.setValue("Loaded One Ring (Alert)")
        log("*** Loaded One Ring (Alert)")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.ringMagazineFull()
        ledPattern.setValue("Ring Magazine Full (Alert)")
        log("*** Ring Magazine Full (Alert)")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show shooting alerts") {
        // *** SHOOTING ***
        log("*** Lighting Test: Shooting ***")
        ledUtil.outOfRange()
        ledPattern.setValue("Out Of Range")
        log("*** Out Of Range")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.almostReadyToShoot()
        ledPattern.setValue("Almost Ready To Shoot")
        log("*** Almost Ready To Shoot")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.readyToShoot()
        ledPattern.setValue("Ready To Shoot")
        log("*** Ready To Shoot")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show TeleOp feedback") {
        // *** TELEOP MODE FEEDBACK ***
        modeLine.setValue("TeleOp")
        ledUtil.teleOpDefault()
        ledPattern.setValue("TeleOp Default")
        log("*** TeleOp")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show more shooting alerts") {
        // *** SHOOTING ***
        log("*** Lighting Test: Shooting ***")
        ledUtil.outOfRange()
        ledPattern.setValue("Out Of Range")
        log("*** Out Of Range")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.almostReadyToShoot()
        ledPattern.setValue("Almost Ready To Shoot")
        log("*** Almost Ready To Shoot")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.readyToShoot()
        ledPattern.setValue("Ready To Shoot")
        log("*** Ready To Shoot")
        letItRun(opMode).expect("Could not complete run")
        ledUtil.autonomousAction()
        ledPattern.setValue("Autonomous Action During TeleOp")
        log("*** Auto-TeleOp")
        letItRun(opMode).expect("Could not complete run")
    }
    // *** GAME PERIODS ***
    /*
    log("*** Lighting Test: Game Period Defaults ***");
    ledUtil.autonomousDefault();
    ledPattern.setValue("Autonomous Default");
    log("*** Autonomous");

    if (!letItRun(opMode)) { abortTest(); }

    ledUtil.teleOpDefault();
    ledPattern.setValue("TeleOp Default");
    log("*** TeleOp");

    if (!letItRun(opMode)) { abortTest(); }
    */
    it("can show endgame lighting") {
        ledUtil.endgameDefault()
        ledPattern.setValue("Endgame Default")
        log("*** EndGame")
        letItRun(opMode).expect("Could not complete run")
    }

    it("can show game over lighting") {
        ledUtil.gameOver()
        ledPattern.setValue("Game Over")
        log("*** Game Over")
        letItRun(opMode).expect("Could not complete run")
        log("*** Lighting Test: Completed Successfully. ***")
    }
}