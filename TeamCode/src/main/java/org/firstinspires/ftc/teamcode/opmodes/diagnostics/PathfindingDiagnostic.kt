package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.diagnostics.tests.pathfindingTest

@TeleOp(name = "Pathfinding Diagnostic")
class PathfindingDiagnostic : DiagnosticsOpMode(::pathfindingTest) {
    override fun provides(): Array<Subject> = arrayOf(DriveBase)
}