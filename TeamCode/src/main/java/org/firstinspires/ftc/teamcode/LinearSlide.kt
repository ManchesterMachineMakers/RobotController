package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.roundToInt

object LinearSlide : CustomBlocksOpModeCompanion() {
    var lowerLimit: DigitalChannel? = null
    var upperLimit: DigitalChannel? = null
    var drive: DcMotor? = null
    val ticksPerRevolution = 1425.1
    val motorPower = 0.4
    val slowMotorPower = 0.2
    var line: Telemetry.Item? = null
    private lateinit var gamepadManager: GamepadManager;
    
    fun Double.ticks(): Int {
        return (this * ticksPerRevolution).roundToInt()
    }

    val base = 0.0.ticks()
    val toCone = 0.5.ticks()
    val low = 3.5.ticks()
    val mid = 5.5.ticks()
    val high = 8.0.ticks()

    @JvmStatic
    fun initHardware() {
        //TODO: set lower limit switch
        //TODO: set upper limit switch
        drive = hardwareMap.dcMotor.get("slide_drive")
        drive!!.direction = DcMotorSimple.Direction.REVERSE
        drive!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        gamepadManager = GamepadManager(gamepad2)
    }

    override fun exists() = hardwareMap.dcMotor.contains("slide_drive")

    @JvmStatic @ExportToBlocks
    fun goTo(ticks: Int) {
        drive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        (drive as DcMotorEx).targetPositionTolerance = 0
        drive!!.targetPosition = ticks
        drive!!.mode = DcMotor.RunMode.RUN_TO_POSITION
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
        line ?:= telemetry.addData("wasPressed", gamepadManager.wasPressed)
        line?.setValue("wasPressed", gamepadManager.wasPressed)
        gamepadManager.once("dpad_left") { goTo(base) }
        gamepadManager.once("a") { goTo(toCone) }
        gamepadManager.once("x") { goTo(low) }
        gamepadManager.once("y") { goTo(mid) }
        gamepadManager.once("b") { goTo(high) }
        gamepadManager.once("left_bumper") { /* toggle claw */ }
        gamepadManager.on("dpad_up") { power = slowMotorPower }
        gamepadManager.on("dpad_down") { power = -slowMotorPower }
        gamepadManager.off(listOf("dpad_up", "dpad_down")) { power = 0.0 }
    }
}
