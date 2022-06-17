package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.gamepadTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

@TeleOp(name = "Gamepad Diagnostic", group = "Diagnostics")
class Diagnostics_GamepadOnly : DiagnosticsOpMode(::gamepadTest)