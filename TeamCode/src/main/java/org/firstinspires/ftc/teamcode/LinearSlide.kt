package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.roundToInt

object LinearSlide : CustomBlocksOpModeCompanion() {
    var lowerLimit: DigitalChannel? = null;
    var upperLimit: DigitalChannel? = null;
    var drive: DcMotor? = null;
    val ticksPerRevolution = 537.7;
    
    fun Double.ticks(): Int {
        return (this * ticksPerRevolution).roundToInt()
    }

    val base = 0.0.ticks()
    val toCone = 0.5.ticks()
    val low = 3.5.ticks()
    val mid = 5.5.ticks()

    @JvmStatic
    fun initHardware() {
        //TODO: set lower limit switch
        //TODO: set upper limit switch
        drive = hardwareMap.dcMotor.get("slide_drive");
        drive?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        drive?.mode = DcMotor.RunMode.RUN_TO_POSITION;
    }

    @JvmStatic @ExportToBlocks
    fun goToBase() {
        drive?.targetPosition = base
    }

    @JvmStatic @ExportToBlocks
    fun goToCone() {
        drive?.targetPosition = toCone
    }

    @JvmStatic @ExportToBlocks
    fun goToLow() {
        drive?.targetPosition = low
    }

    @JvmStatic @ExportToBlocks
    fun goToMid() {
        drive?.targetPosition = mid
    }

    @JvmStatic @ExportToBlocks
    fun isBusy() = drive?.isBusy ?: false;
}