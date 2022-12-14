package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.roundToInt

object LinearSlide : CustomBlocksOpModeCompanion() {
    var lowerLimit: DigitalChannel? = null;
    var upperLimit: DigitalChannel? = null;
    var drive: DcMotor? = null;
    val ticksPerRevolution = 1425.1;
    val motorPower = 0.4
    
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
        drive = hardwareMap.dcMotor.get("slide_drive")
        drive!!.direction = DcMotorSimple.Direction.REVERSE
        drive!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    override fun exists() = hardwareMap.dcMotor.contains("slide_drive")

    @JvmStatic @ExportToBlocks
    fun goToBase() {
        drive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        (drive as DcMotorEx).targetPositionTolerance = 0
        drive!!.targetPosition = base
        drive!!.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    @JvmStatic @ExportToBlocks
    fun goToCone() {
        drive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        (drive as DcMotorEx).targetPositionTolerance = 0
        drive!!.targetPosition = toCone
        drive!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        drive!!.power = motorPower
    }

    @JvmStatic @ExportToBlocks
    fun goToLow() {
        drive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        (drive as DcMotorEx).targetPositionTolerance = 0
        drive!!.targetPosition = low
        drive!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        drive!!.power = motorPower
    }

    @JvmStatic @ExportToBlocks
    fun goToMid() {
        drive!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        (drive as DcMotorEx).targetPositionTolerance = 0
        drive!!.targetPosition = mid
        drive!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        drive!!.power = motorPower
    }

    @JvmStatic @ExportToBlocks
    fun isBusy() = drive!!.isBusy;
}
