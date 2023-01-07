package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.DriveBase
import org.firstinspires.ftc.teamcode.PowerPlayAutonomous
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.util.pathfinder.Path
import org.firstinspires.ftc.teamcode.util.pathfinder.Segment

fun pathfindingTest(opMode: DiagnosticsOpMode) = describe<DriveBase> { driveBase ->
    runner.log("Running paths from the autonomous")
    it("runs path 1") { driveBase.runPath(PowerPlayAutonomous.path1) }
    it("runs path 2") { driveBase.runPath(PowerPlayAutonomous.path2) }
    it("runs path 0") { driveBase.runPath(PowerPlayAutonomous.path0) }
}