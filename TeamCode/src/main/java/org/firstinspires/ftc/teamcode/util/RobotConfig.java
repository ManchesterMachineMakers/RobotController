package org.firstinspires.ftc.teamcode.util;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad;
import org.firstinspires.ftc.teamcode.util.pathfinder.DistanceSensorManager;
import org.firstinspires.ftc.teamcode.util.pathfinder.IMUManager;
import org.firstinspires.ftc.teamcode.util.pathfinder.Localization;
import org.firstinspires.ftc.teamcode.util.pathfinder.Pathfinder;
import org.firstinspires.ftc.teamcode.util.pathfinder.collision.CollisionDetector;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * Robot hardware configuration.
 */
public class RobotConfig {
    public static final RobotConfig BASE = new RobotConfig() {{
        value("vuforiaKey", "Afbp4I3/////AAABmcEn57recUnKv/3EHsAO+jkAFD02oVRghZ8yX5DjgOIvkxO1ipG/fb3GeprYO+Bp6AVbmvW7ts21c71ldDDS8caXYkWTGpFaJ0CyMMfqJQnUabNsH7/sQjh99KlSOi+dOo75AuLUjuLO3nIksEFYpQ3Q8lAGl0ihH3npeTmO9X9KOTV2NJTNKEXZ3mXxBa8xEs9ZYhQy/ppkpExORmc6R+FJYnyykaTaFaXwdKg/R9LZnPQcLwuDD0EnoYlj74qOwVsekUfKxttKMb+FtFlgYm8pmXI5jqQdyidpSHUQn08G1EvqZBN/iuHWCDVhXP2zFRWcQdTEGltwsg47w/IJuLzFvsz04HEqyBz2Xh9eAbAn");
        value("deliveryCalibrationFile", "calibration_delivery.ser");
        value("detectionTimeoutMillis", 3000);
        value("vuforiaTrackableNames", "FreightFrenzy");
        subassembly(Gamepad.class);
        subassembly(SoundEffects.class);
        subassembly(IMUManager.class);
        subassembly(Localization.class);
        subassembly(CollisionDetector.class);
        subassembly(Pathfinder.class);
    }};

    /**
     * Programming board config
     */
    public static final RobotConfig PROGRAMMING_BOARD = new RobotConfig() {{
        merge(BASE);
        value("configName", "Programming Board");
        subassembly(ProgrammingBoardDriveBase.class);
        subassembly(ActiveIntake.class);
        subassembly(Delivery.class);
        subassembly(Vision.class);
        names(Names_ProgrammingBoard.class);
    }};
    /**
     * Full robot config
     */
    public static final RobotConfig FULL_ROBOT = new RobotConfig() {{
        merge(BASE);
        value("configName", "Full Robot");
        subassembly(MecanumDriveBase.class);
        subassembly(ActiveIntake.class);
        subassembly(Delivery.class);
        subassembly(Blinkin.class);
        subassembly(Vision.class);
        subassembly(DistanceSensorManager.class);
        names(Names_FreightFrenzyRobot.class);
    }};

    public static final RobotConfig PROGRAMMING_BOARD_EXPANDED = new RobotConfig() {{
        merge(FULL_ROBOT);
        value("configName", "Programming Board (Expanded)");
        names(Names_FreightFrenzyProgrammingBoard.class);
    }};


    /**
     * Current config
     * DO NOT EDIT - Set by build matrix script
     */
    public static final RobotConfig CURRENT = PROGRAMMING_BOARD;



    /**
     * A key-value pair for configuration - do not use this directly.
     * @param <K> Key type
     */
    interface ConfigPair<K, V> {
        K key();
        V value() throws Exception;
    }

    public <T extends Subassembly> void subassembly(Class<T> type) {
        config.add(new ConfigPair<Class<T>, Constructor<T>>() {
            @Override
            public Class<T> key() {
                return type;
            }

            @Override
            public Constructor<T> value() throws NoSuchMethodException {
                try {
                    return type.getConstructor(LinearOpMode.class);
                } catch (NoSuchMethodException e) {
                    return type.getConstructor(OpMode.class);
                }
            }
        });
    }

    public <T> void value(String key, T value) {
         config.removeIf(pair -> pair.key() == key);
         config.add(new ConfigPair<String, T>() {
            @Override
            public String key() {
                return key;
            }

            @Override
            public T value() throws Exception {
                return value;
            }
        });
    }

    public <T> void names(T names) {
        value("names", names);
    }

    public void merge(RobotConfig other) {
        config.addAll(other.config);
    }

    private final List<ConfigPair<?, ?>> config = new ArrayList<>();

    /**
     * Whether a configuration has a certain piece of hardware or not.
     * @param hardware The subassembly class to check for
     * @param <T> The subassembly class to check for
     * @return Whether the requested subassembly exists in the configuration.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public <T extends Subassembly> boolean hasHardware(Class<T> hardware) {
        return config.stream()
                .anyMatch(canUseSubassembly(hardware));
    }

    /**
     * Get an instance of a subassembly, if it exists in the configuration
     * @param hardware the requested subassembly
     * @param opMode the current opmode
     * @param <T> the requested subassembly
     * @return the requested subassembly, if it exists; otherwise, null.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public <T extends Subassembly> T getHardware(Class<T> hardware, LinearOpMode opMode) throws NoSuchMethodException {
        for(ConfigPair<?, ?> subassemblyAccessor : config) {
            RobotLog.i("Checking if " + subassemblyAccessor.getClass().getMethod("key").getReturnType().getSimpleName() + " works for " + hardware.getSimpleName());
            if(canUseSubassembly(hardware).test(subassemblyAccessor)) {
                RobotLog.i("Yes");
                try {
                    return (T) ((Constructor<? extends Subassembly>) subassemblyAccessor.value()).newInstance(opMode);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
        return null;
    }

    public <T> T getValue(String key) {
        for(ConfigPair<?, ?> configPair : config) {
            try {
                if(configPair.key() == key) {
                    return (T) configPair.value();
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return null;
    }

    /**
     * Get instances of all subassemblies that are {@link Testable}.
     * @param opMode the current opmode
     * @return instances of all subassemblies that are {@link Testable}.
     * @see org.firstinspires.ftc.teamcode.Diagnostics
     * @see Testable
     * @see org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public Testable[] getTestable(OpMode opMode) {
        return config.stream()
                .filter(canUseSubassembly(Testable.class))
                .map(accessor -> {
                    try {
                        return ((Constructor<? extends Subassembly>) accessor.value()).newInstance(opMode);
                    } catch (Exception e) {
                        e.printStackTrace();
                        return null;
                    }
                })
                .map(Testable.class::cast)
                .toArray(Testable[]::new);
    }

    private <T> Predicate<ConfigPair<?, ?>> canUseSubassembly(Class<T> required) {
        return (ConfigPair<?, ?> accessor) -> {
            if(accessor.key().getClass() == Class.class) {
                Class<?> ret = ((Class<?>)accessor.key());
                RobotLog.i("Checking if " + ret.getCanonicalName() + " is usable for " + required.getName());
                boolean result = required.isAssignableFrom(ret);
                RobotLog.i("Result: " + result);
                return result;
            }
            return false;
        };
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public String name(String id) {
        try {
            return ((String)((Class<?>)config.stream()
                    .filter(cfg -> cfg.key() == "names")
                    .findFirst()
                    .orElseGet(() -> new ConfigPair<String, Class<?>>() {
                        @Override
                        public String key() {
                            return "names";
                        }

                        @Override
                        public Class<?> value() throws Exception {
                            return Names.class;
                        }
                    })
                    .value())
                    .getDeclaredField(id)
                    .get(null));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}
