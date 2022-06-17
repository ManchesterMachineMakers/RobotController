package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.diagnostics.tests.deliveryTest
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

@TeleOp(name = "Delivery Diagnostic", group = "Diagnostics")
class Diagnostics_DeliveryOnly : DiagnosticsOpMode(::deliveryTest)