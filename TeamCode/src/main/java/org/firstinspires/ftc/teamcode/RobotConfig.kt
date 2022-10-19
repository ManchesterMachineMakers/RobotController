package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion

object RobotConfig : BlocksOpModeCompanion() {

    fun initHardwareMaps(hardwareMap: HardwareMap) {
        CustomBlocksOpModeCompanion.setHardwareMap(DriveBase::class.java, hardwareMap)
    }

    fun fullRobot() {
        DriveBase.use(org.firstinspires.ftc.teamcode.util.drivebase.config.mecanum)
    }

    fun programmingBoard() {
        DriveBase.use(org.firstinspires.ftc.teamcode.util.drivebase.config.programmingBoard)
    }

    @ExportToBlocks(
        comment = "Initialize the robot - do this first!"
    )
    @JvmStatic
    fun init() { fullRobot() } // DO NOT CHANGE THIS

    inline fun <reified T> getHardware(name: String): T = hardwareMap.get(T::class.java, name)
}