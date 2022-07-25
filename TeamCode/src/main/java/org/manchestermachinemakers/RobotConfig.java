package org.manchestermachinemakers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotConfig {
    public static HardwareMap hwMap;
    public static void activate(HardwareMap hwMap) {
        RobotConfig.hwMap = hwMap;
    }
}
