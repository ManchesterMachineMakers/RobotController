package org.firstinspires.ftc.teamcode.util.configurations

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.drivebase.DriveBase

fun programmingBoard(hardwareMap: HardwareMap) {
    DriveBase.use(org.firstinspires.ftc.teamcode.util.drivebase.config.programmingBoard)
}