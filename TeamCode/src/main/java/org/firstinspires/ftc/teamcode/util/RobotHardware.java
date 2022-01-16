package org.firstinspires.ftc.teamcode.util;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.util.Arrays;

/**
 * Robot hardware configuration.
 */
public interface RobotHardware {
    /**
     * Programming board config
     */
    RobotHardware PROGRAMMING_BOARD = () -> new SubassemblyAccessor<?>[] {
            ProgrammingBoardDriveBase::new,
            Vision::new
    };
    /**
     * Full robot config
     */
    RobotHardware FULL_ROBOT = () -> new SubassemblyAccessor<?>[] {
            MecanumDriveBase::new,
            Delivery::new,
            Blinkin::new,
            Vision::new
    };
    /**
     * Current config
     * DO NOT EDIT - Set by build matrix script
     */
    RobotHardware CURRENT = PROGRAMMING_BOARD;

    /**
     * A functional interface for subassemblies - implemented by subassembly constructors
     * @param <T> The type of the subassembly
     */
    interface SubassemblyAccessor<T extends Subassembly> {
        T get(OpMode opMode);
    }

    /**
     * A list of subassemblies, implemented by each config
     * @return The subassemblies available for that configuration
     */
    SubassemblyAccessor<? extends Subassembly>[] subassemblies();

    /**
     * Whether a configuration has a certain piece of hardware or not.
     * @param hardware The subassembly class to check for
     * @param <T> The subassembly class to check for
     * @return Whether the requested subassembly exists in the configuration.
     */
    default <T extends Subassembly> boolean has(Class<T> hardware) {
        for (SubassemblyAccessor<?> accessor:
             subassemblies()) {
            try {
                Method get = accessor.getClass().getMethod("get");
                if(hardware.isAssignableFrom(get.getReturnType())) {
                    return true;
                }
            } catch (NoSuchMethodException ignored) {
                return false;
            }
        }
        return false;
    }

    /**
     * Get an instance of a subassembly, if it exists in the configuration
     * @param hardware the requested subassembly
     * @param opMode the current opmode
     * @param <T> the requested subassembly
     * @return the requested subassembly, if it exists; otherwise, null.
     */
    default <T extends Subassembly> T get(Class<T> hardware, OpMode opMode) {
        for (SubassemblyAccessor<?> accessor:
                subassemblies()) {
            try {
                Method get = accessor.getClass().getMethod("get");
                if(hardware.isAssignableFrom(get.getReturnType())) {
                    return (T) accessor.get(opMode);
                }
            } catch (NoSuchMethodException ignored) {
                return null;
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
        return Arrays.stream(subassemblies())
                .filter(accessor -> {
                    try {
                        return Testable.class.isAssignableFrom(accessor.getClass().getMethod("get").getReturnType());
                    } catch (NoSuchMethodException e) {
                        e.printStackTrace();
                        return false;
                    }
                })
                .map(accessor -> accessor.get(opMode))
                .map(Testable.class::cast)
                .toArray(Testable[]::new);
    }
}
