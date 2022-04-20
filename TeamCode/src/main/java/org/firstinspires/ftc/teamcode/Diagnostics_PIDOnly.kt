package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.pidTest

@TeleOp(name = "PID Controller Diagnostic")
class Diagnostics_PIDOnly : DiagnosticsOpMode(::pidTest)