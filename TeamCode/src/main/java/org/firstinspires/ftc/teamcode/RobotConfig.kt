package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.ExportToBlocks

object RobotConfig : BlocksOpModeCompanion() {

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