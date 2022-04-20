package org.firstinspires.ftc.teamcode.util

import kotlin.Throws
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.RobotConfig.ConfigPair
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.util.RobotConfig
import androidx.annotation.RequiresApi
import android.os.Build
import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
import org.firstinspires.ftc.teamcode.util.SoundEffects
import org.firstinspires.ftc.teamcode.util.pathfinder.IMUManager
import org.firstinspires.ftc.teamcode.util.pathfinder.Localization
import org.firstinspires.ftc.teamcode.util.pathfinder.collision.CollisionDetector
import org.firstinspires.ftc.teamcode.util.pathfinder.Pathfinder
import org.firstinspires.ftc.teamcode.drivebase.ProgrammingBoardDriveBase
import org.firstinspires.ftc.teamcode.sensors.Vision
import org.firstinspires.ftc.teamcode.util.Names_ProgrammingBoard
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.*
import org.firstinspires.ftc.teamcode.util.pathfinder.DistanceSensorManager
import org.firstinspires.ftc.teamcode.util.Names_FreightFrenzyRobot
import org.firstinspires.ftc.teamcode.util.Names_FreightFrenzyProgrammingBoard
import java.lang.Exception
import java.lang.reflect.Constructor
import java.util.ArrayList
import java.util.function.Predicate

/**
 * Robot hardware configuration.
 */
open class RobotConfig {
    /**
     * A key-value pair for configuration - do not use this directly.
     * @param <K> Key type
    </K> */
    internal interface ConfigPair<K, V> {
        fun key(): K

        @Throws(Exception::class)
        fun value(): V
    }

    fun <T : Subassembly?> subassembly(type: Class<T>) {
        config.add(type to
            try {
                type.getConstructor(LinearOpMode::class.java)
            } catch (e: NoSuchMethodException) {
                type.getConstructor(OpMode::class.java)
            }
        )
    }

    fun <T> value(key: String, value: T) {
        config.removeIf { pair: Pair<*, *> -> pair.first === key }
        config.add(key to value)
    }

    fun <T> names(names: T) {
        value("names", names)
    }

    fun merge(other: RobotConfig) {
        config.addAll(other.config)
    }

    private val config: MutableList<Pair<*, *>> = ArrayList()

    /**
     * Whether a configuration has a certain piece of hardware or not.
     * @param hardware The subassembly class to check for
     * @param <T> The subassembly class to check for
     * @return Whether the requested subassembly exists in the configuration.
    </T> */
    @RequiresApi(api = Build.VERSION_CODES.N)
    fun <T : Subassembly?> hasHardware(hardware: Class<T>): Boolean {
        return config.stream()
                .anyMatch(canUseSubassembly(hardware))
    }

    /**
     * Get an instance of a subassembly, if it exists in the configuration
     * @param hardware the requested subassembly
     * @param opMode the current opmode
     * @param <T> the requested subassembly
     * @return the requested subassembly, if it exists; otherwise, null.
    </T> */
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Throws(NoSuchMethodException::class)
    fun <T : Subassembly?> getHardware(hardware: Class<T>, opMode: LinearOpMode?): T? {
        for (subassemblyAccessor in config) {
            RobotLog.i("Checking if " + subassemblyAccessor.javaClass.getMethod("key").returnType.simpleName + " works for " + hardware.simpleName)
            if (canUseSubassembly(hardware)(subassemblyAccessor)) {
                RobotLog.i("Yes")
                try {
                    return (subassemblyAccessor.second as Constructor<*>).newInstance(opMode) as T
                } catch (e: Exception) {
                    e.printStackTrace()
                }
            }
        }
        return null
    }

    fun <T> getValue(key: String): T? {
        for (configPair in config) {
            try {
                if (configPair.first === key) {
                    return configPair.second as T
                }
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
        return null
    }

    /**
     * Get instances of all subassemblies that are [Testable].
     * @param opMode the current opmode
     * @return instances of all subassemblies that are [Testable].
     * @see org.firstinspires.ftc.teamcode.Diagnostics
     *
     * @see Testable
     *
     * @see org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    fun getTestable(opMode: OpMode?): Array<Subject> {
        return config
                .filter(canUseSubassembly(Subject::class.java))
                .map { accessor: Pair<*, *> ->
                    try {
                        return@map (accessor.second as Constructor<*>).newInstance(opMode)
                    } catch (e: Exception) {
                        e.printStackTrace()
                        return@map null
                    }
                }
                .mapNotNull { obj: Any? -> Subject::class.java.cast(obj) }
                .toTypedArray()
    }

    private fun <T> canUseSubassembly(required: Class<T>): (Pair<*, *>) -> Boolean {
        return fun(accessor: Pair<*, *>): Boolean {
            if (accessor.first?.javaClass ?: Object::class.java == Class::class.java) {
                val ret = accessor.second as Class<*>
                RobotLog.i("Checking if " + ret.canonicalName + " is usable for " + required.name)
                val result = required.isAssignableFrom(ret)
                RobotLog.i("Result: $result")
                return result
            }
            return false
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    fun name(id: String?): String? {
        return try {
            (config.stream()
                    .filter { cfg: Pair<*, *> -> cfg.first === "names" }
                    .findFirst()
                    .orElseGet {
                        "names" to Names::class.java
                    }
                    .second as Class<*>)
                    .getDeclaredField(id!!)[null] as String
        } catch (e: Exception) {
            e.printStackTrace()
            null
        }
    }

    companion object {
        private fun config(lambda: RobotConfig.() -> Unit) = object : RobotConfig() {
            init {
                this.lambda()
            }
        }

        val BASE = config {
            value("vuforiaKey", "Afbp4I3/////AAABmcEn57recUnKv/3EHsAO+jkAFD02oVRghZ8yX5DjgOIvkxO1ipG/fb3GeprYO+Bp6AVbmvW7ts21c71ldDDS8caXYkWTGpFaJ0CyMMfqJQnUabNsH7/sQjh99KlSOi+dOo75AuLUjuLO3nIksEFYpQ3Q8lAGl0ihH3npeTmO9X9KOTV2NJTNKEXZ3mXxBa8xEs9ZYhQy/ppkpExORmc6R+FJYnyykaTaFaXwdKg/R9LZnPQcLwuDD0EnoYlj74qOwVsekUfKxttKMb+FtFlgYm8pmXI5jqQdyidpSHUQn08G1EvqZBN/iuHWCDVhXP2zFRWcQdTEGltwsg47w/IJuLzFvsz04HEqyBz2Xh9eAbAn")
            value("deliveryCalibrationFile", "calibration_delivery.ser")
            value("detectionTimeoutMillis", 3000)
            value("vuforiaTrackableNames", "FreightFrenzy")
            subassembly(Gamepad::class.java)
            subassembly(SoundEffects::class.java)
            subassembly(IMUManager::class.java)
            subassembly(Localization::class.java)
            subassembly(CollisionDetector::class.java)
            subassembly(Pathfinder::class.java)
            subassembly(DuckySpinner::class.java)
        }

        /**
         * Programming board config
         */
        val PROGRAMMING_BOARD = config {
            merge(BASE)
            value("configName", "Programming Board")
            subassembly(ProgrammingBoardDriveBase::class.java)
            subassembly(ActiveIntake::class.java)
            subassembly(Delivery::class.java)
            subassembly(Vision::class.java)
            names(Names_ProgrammingBoard::class.java)
        }

        /**
         * Full robot config
         */
        val FULL_ROBOT = config {
            merge(BASE)
            value("configName", "Full Robot")
            subassembly(MecanumDriveBase::class.java)
            subassembly(ActiveIntake::class.java)
            subassembly(Delivery::class.java)
            subassembly(Blinkin::class.java)
            subassembly(Vision::class.java)
            subassembly(DistanceSensorManager::class.java)
            names(Names_FreightFrenzyRobot::class.java)
        }
        val PROGRAMMING_BOARD_EXPANDED = config {
            merge(FULL_ROBOT)
            value("configName", "Programming Board (Expanded)")
            names(Names_FreightFrenzyProgrammingBoard::class.java)
        }

        /**
         * Current config
         * DO NOT EDIT - Set by build matrix script
         */
        @JvmField
        val CURRENT = PROGRAMMING_BOARD
    }
}