package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.util.Names;
import org.firstinspires.ftc.teamcode.util.Subassembly;

/**
 * Set different patterns and colors based on the robot status.  Each must be called from within the opMode or status utility.
 */
public class Blinkin implements Subassembly {

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public int alertLengthMilliseconds = 500;

    /**
     * initialize the LED strip from the hardware map
     * @param opMode
     */
    public Blinkin(OpMode opMode) {
        blinkinLedDriver = opMode.hardwareMap.get(RevBlinkinLedDriver.class, Names.servo_BlinkinLED);
    }

    /**
     * Game over! Turn the lights off.
     */
    public void gameOver() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Default pattern during Autonomous period
     */
    public void autonomousDefault() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Autonomous control during TeleOp period - Steady
     */
    public void autonomousAction() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
        blinkinLedDriver.setPattern(pattern);
    }
    /**
     * Autonomous control during TeleOp period - Alert
     */
    public void autonomousActionAlert() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        alertThenReset();
    }
    /**
     * Default pattern during TeleOp period
     */
    public void teleOpDefault() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Default pattern during Endgame period
     */
    public void endgameDefault() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Indicate the number of rings detected.
     * @param numberOfRings
     */
    public void detectedRings(int numberOfRings) {
        RevBlinkinLedDriver.BlinkinPattern ringPattern;
        switch(numberOfRings) {
            case 0:
                ringPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW;
                break;
            case 1:
                ringPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM;
                break;
            case 4:
                ringPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST;
                break;
            default:
                ringPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW;
        }
        blinkinLedDriver.setPattern(ringPattern);
        alertThenReset();
    }

    /**
     * Sets the light pattern to RED, WHITE, or BLUE; or defaults back to the current active pattern.
     * @param lineColor
     */
    public void detectedLine(int lineColor) {
        RevBlinkinLedDriver.BlinkinPattern linePattern;
        switch(lineColor) {
            case Color.RED:
                linePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                break;
            case Color.WHITE:
                linePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                break;
            case Color.BLUE:
                linePattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
                break;
            default:
                linePattern = pattern;
        }
        blinkinLedDriver.setPattern(linePattern);
        alertThenReset();
    }

    /**
     * When the magazine has at least one ring, and
     * the shooter is behind the launch line and angled correctly according to the navigation reckoning, and
     * the shooter motor is up to launch speed.
     */
    public void readyToShoot() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Use this when we're getting close to shooting range,
     * and we have at least one ring on the bot.
     */
    public void almostReadyToShoot() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Use this when we have at least one ring, but we are
     * not in a position to shoot.
     */
    public void outOfRange() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * We detected a ring in the intake.
     */
    public void loadedOneRing() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        alertThenReset();
    }

    /**
     * When the magazine holds less than three rings and is not ready to shoot.
     * As long as there is a possibility of shooting, the readyToShoot pattern should be active.
     */
    public void needMoreRings() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        alertThenReset();
    }

    /**
     * When the bot holds three rings and is not ready to shoot.
     * As long as there is a possibility of shooting, the readyToShoot pattern should be active.
     */
    public void ringMagazineFull() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        alertThenReset();
    }

    /**
     * The bot has grabbed a wobble goal.
     */
    public void grabbedWobbleGoal() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        alertThenReset();
    }


    /**
     * Keep the current pattern for the default time, then reset to the saved one.
     */
    private void alertThenReset() {
        try {
            Thread.sleep(alertLengthMilliseconds);
        } catch (InterruptedException ex) {
            // do nothing, just move on.
        }
        blinkinLedDriver.setPattern(pattern);
    }

}
