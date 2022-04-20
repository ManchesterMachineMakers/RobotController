package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.driveBaseTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

@TeleOp(name = "Mecanum Drive Base Diagnostic", group = "Diagnostics")
class Diagnostics_MecanumBaseOnly : DiagnosticsOpMode(::driveBaseTest)