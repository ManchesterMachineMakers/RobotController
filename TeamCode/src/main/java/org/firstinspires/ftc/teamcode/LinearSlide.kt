package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.util.equalsTolerance
import kotlin.math.roundToInt

object LinearSlide : CustomBlocksOpModeCompanion() {
    // lateinit var lowerLimit: DigitalChannel
    // lateinit var upperLimit: DigitalChannel
    // lateinit var touchSensor: TouchSensor
    var drive: DcMotor? = null
    var clawServo: Servo? = null
    val ticksPerRevolution = 1425.1
    val motorPower = 0.8
    val slowMotorPower = 0.4
    val clawClosedPos = 0.336
    val clawOpenPos = 0.239
    private lateinit var gamepadManager: GamepadManager;
    
    fun Double.ticks(): Int {
        return (this * ticksPerRevolution).roundToInt()
    }

    val base = 0.0.ticks()
    val toCone = 0.47.ticks()
    val low = 3.5.ticks()
    val mid = 5.65.ticks()
    val high = 8.0.ticks()

    override fun initHardware() {
        // lowerLimit = hardwareMap.digitalChannel.get("")
        // upperLimit = hardwareMap.digitalChannel.get("")
        // touchSensor = hardwareMap.touchSensor.get("")
        drive = hardwareMap.dcMotor.get("slide_drive")
        drive!!.direction = DcMotorSimple.Direction.REVERSE
        drive!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        clawServo = hardwareMap.servo.get("claw_servo")
        openClaw()

        gamepadManager = GamepadManager(gamepad2)
    }

    fun openClaw() {
        clawServo?.position = clawOpenPos
    }

    fun closeClaw() {
        clawServo?.position = clawClosedPos
    }

    fun toggleClaw() {
        if(clawServo?.position?.equalsTolerance(clawOpenPos, 0.05) == true) closeClaw()
        else openClaw()
    }

    override fun exists() = hardwareMap.dcMotor.contains("slide_drive")

    @JvmStatic @ExportToBlocks
    fun goTo(ticks: Int) {
        // if ((ticks < drive!!.currentPosition && lowerLimit.state) || (ticks > drive!!.currentPosition && upperLimit.state)) return;
        drive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        (drive as DcMotorEx).targetPositionTolerance = 1
        drive!!.targetPosition = ticks
        drive!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        drive!!.power = if (ticks < drive!!.currentPosition) slowMotorPower else motorPower // make it harder to break the slides going down
    }

    var power: Double
        set(newPower) {
            drive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
            drive!!.power = newPower
        }
        get() = drive!!.power

    @JvmStatic @ExportToBlocks
    fun isBusy() = drive!!.isBusy;

    @JvmStatic @ExportToBlocks
    fun controller() {
        // dpad left - base
        // dpad up - go up slowly
        // dpad down - go down slowly
        // a - cone
        // x - low
        // y - mid
        // b - high
        // TODO: use triggers/bumpers for manual control
        gamepadManager.once("dpad_left") { goTo(base) }
        gamepadManager.once("a") { goTo(toCone) }
        gamepadManager.once("x") { goTo(low) }
        gamepadManager.once("y") { goTo(mid) }
        gamepadManager.once("b") { goTo(high) }
        gamepadManager.once("left_bumper") { toggleClaw() }
        gamepadManager.on("dpad_up") { power = slowMotorPower }
        gamepadManager.on("dpad_down") { power = -slowMotorPower }
        gamepadManager.off(listOf("dpad_up", "dpad_down")) { power = 0.0 }
        // if((upperLimit.state && power > 0) || (lowerLimit.state && power < 0)) power = 0.0
        // if(touchSensor.isPressed && drive!!.targetPosition < drive!!.currentPosition) drive!!.targetPosition = drive!!.currentPosition;
    }
}
