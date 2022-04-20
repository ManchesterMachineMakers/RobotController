package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.duckyVisionTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

@TeleOp(name = "Vision Diagnostic", group = "Diagnostics")
class Diagnostics_Vision : DiagnosticsOpMode(::duckyVisionTest)