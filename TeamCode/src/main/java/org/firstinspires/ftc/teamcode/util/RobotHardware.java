package org.firstinspires.ftc.teamcode.util;

import android.os.Build;
import android.util.Log;

import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase;
import org.firstinspires.ftc.teamcode.sensors.Vision;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.util.Arrays;
import java.util.function.Predicate;

/**
 * Robot hardware configuration.
 */
public interface RobotHardware {
    /**
     * Programming board config
     */
    RobotHardware PROGRAMMING_BOARD = () -> new SubassemblyAccessor<?>[] {
            ProgrammingBoardDriveBase::new,
            ActiveIntake::new,
            Vision::new
    };
    /**
     * Full robot config
     */
    RobotHardware FULL_ROBOT = () -> new SubassemblyAccessor<?>[] {
            MecanumDriveBase::new,
            ActiveIntake::new,
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
    @RequiresApi(api = Build.VERSION_CODES.N)
    default <T extends Subassembly> boolean has(Class<T> hardware) {
        return Arrays.stream(subassemblies())
                .anyMatch(RobotHardware.canUseSubassembly(hardware));
    }

    /**
     * Get an instance of a subassembly, if it exists in the configuration
     * @param hardware the requested subassembly
     * @param opMode the current opmode
     * @param <T> the requested subassembly
     * @return the requested subassembly, if it exists; otherwise, null.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    default <T extends Subassembly> T get(Class<T> hardware, OpMode opMode) throws NoSuchMethodException {
        for(SubassemblyAccessor<?> subassemblyAccessor : subassemblies()) {
            RobotLog.i("Checking if " + subassemblyAccessor.getClass().getMethod("get", OpMode.class).getReturnType().getSimpleName() + " works for " + hardware.getSimpleName());
            if(RobotHardware.canUseSubassembly(hardware).test(subassemblyAccessor)) {
                RobotLog.i("Yes");
                return (T) subassemblyAccessor.get(opMode);
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
                .filter(RobotHardware.canUseSubassembly(Testable.class))
                .map(accessor -> accessor.get(opMode))
                .map(Testable.class::cast)
                .toArray(Testable[]::new);
    }

    static <T> Predicate<SubassemblyAccessor<?>> canUseSubassembly(Class<T> required) {
        return (SubassemblyAccessor<?> accessor) -> {
            try {
                Class<?> ret = accessor.getClass().getMethod("get", OpMode.class).getReturnType();
                RobotLog.i("Checking if " + ret.getCanonicalName() + " is usable for " + required.getName());
                boolean result = required.isAssignableFrom(ret);
                RobotLog.i("Result: " + result);
                return result;
            } catch (NoSuchMethodException e) {
                e.printStackTrace();
                return false;
            }
        };
    }
}
