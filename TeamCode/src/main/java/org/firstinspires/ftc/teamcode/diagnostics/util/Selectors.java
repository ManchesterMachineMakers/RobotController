package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

public interface Selectors {
    Selector<DriveBase> driveBaseSelector();
    Selector<Blinkin> lightingSelector();
    Selector<Delivery> deliverySelector();
    Selector<ActiveIntake> activeIntakeSelector();
    Selector<Vision> cameraSelector();
}
