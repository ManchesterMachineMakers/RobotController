package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.deliveryControllerTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

@TeleOp(name = "Delivery Controller Diagnostic", group = "Diagnostics")
class Diagnostics_DeliveryControllerOnly : DiagnosticsOpMode(::deliveryControllerTest)