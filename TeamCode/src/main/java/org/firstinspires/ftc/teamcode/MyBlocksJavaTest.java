package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class MyBlocksJavaTest extends BlocksOpModeCompanion {
    @ExportToBlocks(comment = "A mystery method. Do not use in competition.")
    public static void sayHello() {
        telemetry.speak("Hello, World!");
    }
}
