package org.firstinspires.ftc.teamcode.util;

import android.os.Build;

import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

import java.lang.reflect.Constructor;
import java.util.Arrays;
import java.util.function.Predicate;
import java.util.stream.Stream;

/**
 * Robot hardware configuration.
 */
public interface RobotConfig {
    /**
     * Programming board config
     */
    RobotConfig PROGRAMMING_BOARD = () -> new ConfigPair<?, ?>[] {
            value("configName", "Programming Board"),
            subassembly(ProgrammingBoardDriveBase.class),
            subassembly(ActiveIntake.class),
            subassembly(Vision.class),
            names(Names_ProgrammingBoard.class)
    };
    /**
     * Full robot config
     */
    RobotConfig FULL_ROBOT = () -> new ConfigPair<?, ?>[] {
            value("configName", "Full Robot"),
            subassembly(MecanumDriveBase.class),
            subassembly(ActiveIntake.class),
            subassembly(Delivery.class),
            subassembly(Blinkin.class),
            subassembly(Vision.class),
            names(Names_FreightFrenzyRobot.class)
    };


    /**
     * Current config
     * DO NOT EDIT - Set by build matrix script
     */
    RobotConfig CURRENT = PROGRAMMING_BOARD;



    /**
     * A key-value pair for configuration - do not use this directly.
     * @param <K> Key type
     */
    interface ConfigPair<K, V> {
        K key();
        V value() throws Exception;
    }

    static <T extends Subassembly> ConfigPair<Class<T>, Constructor<T>> subassembly(Class<T> type) {
        return new ConfigPair<Class<T>, Constructor<T>>() {
            @Override
            public Class<T> key() {
                return type;
            }

            @Override
            public Constructor<T> value() throws NoSuchMethodException {
                return type.getConstructor(OpMode.class);
            }
        };
    }

    static <T> ConfigPair<String, T> value(String key, T value) {
        return new ConfigPair<String, T>() {
            @Override
            public String key() {
                return key;
            }

            @Override
            public T value() throws Exception {
                return value;
            }
        };
    }

    static <T> ConfigPair<String, T> names(T names) {
        return value("names", names);
    }

    /**
     * A list of configuration
     * @return The subassemblies available for that configuration
     */
    ConfigPair<?, ?>[] config();

    /**
     * Whether a configuration has a certain piece of hardware or not.
     * @param hardware The subassembly class to check for
     * @param <T> The subassembly class to check for
     * @return Whether the requested subassembly exists in the configuration.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    default <T extends Subassembly> boolean hasHardware(Class<T> hardware) {
        return Arrays.stream(config())
                .anyMatch(RobotConfig.canUseSubassembly(hardware));
    }

    /**
     * Get an instance of a subassembly, if it exists in the configuration
     * @param hardware the requested subassembly
     * @param opMode the current opmode
     * @param <T> the requested subassembly
     * @return the requested subassembly, if it exists; otherwise, null.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    default <T extends Subassembly> T getHardware(Class<T> hardware, OpMode opMode) throws NoSuchMethodException {
        for(ConfigPair<?, ?> subassemblyAccessor : config()) {
            RobotLog.i("Checking if " + subassemblyAccessor.getClass().getMethod("key").getReturnType().getSimpleName() + " works for " + hardware.getSimpleName());
            if(RobotConfig.canUseSubassembly(hardware).test(subassemblyAccessor)) {
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

    default <T> T getValue(String key) {
        for(ConfigPair<?, ?> configPair : config()) {
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
    default Testable[] getTestable(OpMode opMode) {
        return Arrays.stream(config())
                .filter(RobotConfig.canUseSubassembly(Testable.class))
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

    static <T> Predicate<ConfigPair<?, ?>> canUseSubassembly(Class<T> required) {
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
    default String name(String id) {
        try {
            return ((String)((Class<?>)Stream.of(this.config())
                    .filter(cfg -> cfg.key() == "names")
                    .findFirst()
                    .orElseGet(() -> names(Names.class))
                    .value())
                    .getDeclaredField(id)
                    .get(null));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}
