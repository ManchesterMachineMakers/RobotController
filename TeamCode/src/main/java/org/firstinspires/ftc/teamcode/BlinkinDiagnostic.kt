package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.diagnostics.tests.blinkinTest

@TeleOp(name = "Blinkin Diagnostic", group = "Diagnostics")
class BlinkinDiagnostic : DiagnosticsOpMode(::blinkinTest)