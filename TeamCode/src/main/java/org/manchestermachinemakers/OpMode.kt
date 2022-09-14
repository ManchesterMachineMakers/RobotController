package org.manchestermachinemakers

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.manchestermachinemakers.hardware.drivebase.DriveBase
import org.manchestermachinemakers.hardware.drivebase.config.programmingBoard

abstract class OpMode : LinearOpMode() {
    override fun runOpMode() {
        RobotConfig.activate(hardwareMap)
        DriveBase.use(programmingBoard)
        run()
    }

    abstract fun run()
}