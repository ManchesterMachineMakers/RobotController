package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;

@Test("Another Drive Base Test")
@Requires(DriveBase.class)
public class DriveBaseTest2 implements Base {

    @Override
    public boolean run(Testable[] sel, Runner runner) throws Exception {
        DriveBase driveBase = Testable.getOrDie(sel, DriveBase.class);
        driveBase.go(0.2);
        return true;
    }
}
