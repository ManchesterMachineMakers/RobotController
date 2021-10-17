package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.WobbleGoalGrabber;

public interface Selectors {
    Selector<DriveBase> driveBaseSelector();
    Selector<Blinkin> lightingSelector();
    Selector<WobbleGoalGrabber> wggSelector();
    Selector<ActiveIntake> activeIntakeSelector();
}
