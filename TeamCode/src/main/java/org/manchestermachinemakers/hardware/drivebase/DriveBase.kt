package org.manchestermachinemakers.hardware.drivebase

import org.manchestermachinemakers.hardware.drivebase.DriveBase
import org.manchestermachinemakers.hardware.drivebase.DriveBase.TravelDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import org.manchestermachinemakers.RobotConfig

object DriveBase {
    private var config: Configuration? = null

    @JvmStatic
    fun use(config: Configuration) {
        DriveBase.config = config
    }

    /**
     * Constants to represent drive speeds that are useful.  Set power values for each in initDriveSpeedConfiguration().
     */
    enum class DriveSpeed {
        STOP, ANGLE_CORRECTION, SLOW, FAST, MAX
    }

    /**
     * A predefined set of travel directions. Set motor configurations for each direction in Configuration.init().
     */
    enum class TravelDirection {
        base, forward, reverse, pivotLeft, pivotRight, strafeLeft, strafeLeftForward, strafeLeftBackward, strafeRight, strafeRightForward, strafeRightBackward, pitch
    }

    class Configuration(motors: Array<String>, val config: HashMap<TravelDirection, Array<DcMotorSimple.Direction?>>) {
        val motors = motors.map { name -> RobotConfig.getHardware<DcMotor>(name) }
    }
}