package org.firstinspires.ftc.teamcode
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion
import com.qualcomm.hardware.rev.RevBlinkinLedDriver

class Blinkin : CustomBlocksOpModeCompanion() {
    lateinit var blinkinLedDriver: RevBlinkinLedDriver;
    var pattern: RevBlinkinLedDriver.BlinkinPattern? = null;
    val alertLengthMilliseconds = 500L;

    override fun initHardware() {
        blinkinLedDriver = opMode.hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkinLED")
    }

    override fun exists()
        = hardwareMap.servo.contains("blinkinLED")

    /**
     * Game over! Turn the lights off.
     */
    fun gameOver() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Default pattern during Autonomous period
     */
    fun autonomousDefault() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Autonomous control during TeleOp period - Steady
     */
    fun autonomousAction() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
        blinkinLedDriver.setPattern(pattern);
    }
    /**
     * Autonomous control during TeleOp period - Alert
     */
    fun autonomousActionAlert() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
        alertThenReset();
    }
    /**
     * Default pattern during TeleOp period
     */
    fun teleOpDefault() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Default pattern during Endgame period
     */
    fun endgameDefault() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * Currently detecting things
     */
    fun detecting() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
        alertThenReset();
    }

    /**
     * Keep the current pattern for the default time, then reset to the saved one.
     */
    fun alertThenReset() {
        try {
            Thread.sleep(alertLengthMilliseconds);
        } catch (ex: InterruptedException) {
            // do nothing, just move on.
        }
        blinkinLedDriver.setPattern(pattern);
    }
}