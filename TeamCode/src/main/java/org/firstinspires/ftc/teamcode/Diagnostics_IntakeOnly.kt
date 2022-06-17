package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.intakeTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

@TeleOp(name = "Intake Diagnostic", group = "Diagnostics")
class Diagnostics_IntakeOnly : DiagnosticsOpMode(::intakeTest)