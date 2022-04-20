package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.lightingTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

@TeleOp(name = "Lighting Diagnostic", group = "Diagnostics")
class Diagnostics_LightingOnly : DiagnosticsOpMode(::lightingTest)