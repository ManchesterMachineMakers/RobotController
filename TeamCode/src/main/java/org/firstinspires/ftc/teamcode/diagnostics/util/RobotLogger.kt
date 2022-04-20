package org.firstinspires.ftc.teamcode.diagnostics.util

import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Logger

class RobotLogger : Logger {
    override fun log(what: String) {
        RobotLog.i(what)
    }
}