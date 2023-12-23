package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Logger

class RobotLogger : Logger {
    override fun log(what: String) {
        RobotLog.i("diagonal: $what")
    }

}