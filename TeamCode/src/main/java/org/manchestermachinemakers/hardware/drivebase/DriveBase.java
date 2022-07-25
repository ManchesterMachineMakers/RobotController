package org.manchestermachinemakers.hardware.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.manchestermachinemakers.RobotConfig;

import java.util.Arrays;
import java.util.HashMap;

public class DriveBase {
    /**
     * Constants to represent drive speeds that are useful.  Set power values for each in initDriveSpeedConfiguration().
     */
    public enum DriveSpeed {
        STOP,
        ANGLE_CORRECTION,
        SLOW,
        FAST,
        MAX
    }

    /**
     * A predefined set of travel directions. Set motor configurations for each direction in Configuration.init().
     */
    public enum TravelDirection {
        base,

        forward,
        reverse,
        pivotLeft,
        pivotRight,

        strafeLeft,
        strafeLeftForward,
        strafeLeftBackward,

        strafeRight,
        strafeRightForward,
        strafeRightBackward,
        pitch
    }


    public static abstract class Configuration {
        public HashMap<TravelDirection, DcMotorSimple.Direction[]> config;
        public DcMotor[] motors;
        public abstract void init();
        public void put(TravelDirection key, DcMotorSimple.Direction... directions) {
            config.put(key, directions);
        }
        public void setMotors(String... motors) {
            this.motors = (DcMotor[]) Arrays.stream(motors).map(motor -> RobotConfig.hwMap.get(DcMotor.class, motor)).toArray();
        }
    }

    private static Configuration config;

    public static void use(Configuration config) {
        DriveBase.config = config;
    }
}
