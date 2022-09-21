package org.firstinspires.ftc.teamcode.util.configurations

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.drivebase.DriveBase

fun fullRobot(hardwareMap: HardwareMap) {
    DriveBase.use(org.firstinspires.ftc.teamcode.util.drivebase.config.programmingBoard)
}