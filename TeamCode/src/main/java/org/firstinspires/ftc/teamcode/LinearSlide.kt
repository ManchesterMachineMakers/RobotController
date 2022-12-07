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
    val maxHeight: Int = (5 * ticksPerRevolution).roundToInt();

    @JvmStatic
    fun initHardware() {
        //TODO: set lower limit switch
        //TODO: set upper limit switch
        drive = hardwareMap.dcMotor.get("slide_drive");
        drive?.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        drive?.mode = DcMotor.RunMode.RUN_TO_POSITION;
    }

    @JvmStatic @ExportToBlocks
    fun goToTop() {
        drive?.targetPosition = maxHeight;
    }

    @JvmStatic @ExportToBlocks
    fun goToBottom() {
        drive?.targetPosition = 0;
    }

    @JvmStatic @ExportToBlocks
    fun isBusy() = drive?.isBusy ?: false;
}