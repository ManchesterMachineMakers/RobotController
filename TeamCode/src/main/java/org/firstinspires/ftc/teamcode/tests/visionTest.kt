package org.firstinspires.ftc.teamcode.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.opmodes.diagnostics.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Vision

fun visionTest(opMode: DiagnosticsOpMode) = describe<Vision> { vision ->
    it("initializes") {
        (vision.aprilTag != null && vision.visionPortal != null).expect("Vision failed to initialize")
    }
}