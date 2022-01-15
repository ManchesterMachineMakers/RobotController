package org.firstinspires.ftc.teamcode.util;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

import java.lang.reflect.Method;
import java.util.Arrays;

/**
 * Robot hardware configuration.
 */
public interface RobotHardware {
    /**
     * Programming board config
     */
    RobotHardware PROGRAMMING_BOARD = () -> new SubassemblyAccessor<?>[] {
            ProgrammingBoardDriveBase::new
    };
    /**
     * Full robot config
     */
    RobotHardware FULL_ROBOT = () -> new SubassemblyAccessor<?>[] {
            MecanumDriveBase::new,
            Delivery::new,
            Blinkin::new
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
        T get(HardwareMap hwMap);
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
     * @param hwMap the hardware map to pass to the subassembly
     * @param <T> the requested subassembly
     * @return the requested subassembly, if it exists; otherwise, null.
     */
    default <T extends Subassembly> T get(Class<T> hardware, HardwareMap hwMap) {
        for (SubassemblyAccessor<?> accessor:
                subassemblies()) {
            try {
                Method get = accessor.getClass().getMethod("get");
                if(hardware.isAssignableFrom(get.getReturnType())) {
                    return (T) accessor.get(hwMap);
                }
            } catch (NoSuchMethodException ignored) {
                return null;
            }
        }
        return null;
    }

    /**
     * Get instances of all subassemblies that are {@link Testable}.
     * @param hwMap the hardware map to pass to each subassembly
     * @return instances of all subassemblies that are {@link Testable}.
     * @see org.firstinspires.ftc.teamcode.Diagnostics
     * @see Testable
     * @see org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    default Testable[] getTestable(HardwareMap hwMap) {
        return Arrays.stream(subassemblies())
                .map(accessor -> accessor.get(hwMap))
                .filter(subassembly -> Testable.class.isAssignableFrom(subassembly.getClass()))
                .map(Testable.class::cast)
                .toArray(Testable[]::new);
    }
}
