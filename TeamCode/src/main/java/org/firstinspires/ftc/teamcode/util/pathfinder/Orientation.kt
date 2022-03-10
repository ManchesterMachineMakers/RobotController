package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration
import org.firstinspires.ftc.robotcore.external.navigation.Axis
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import kotlin.math.asin
import kotlin.math.atan2

/**
 * Wraps the base IMU and handles the conversion from quaternions to Euler angles. Tweaked from Isaac's code.
 * @param imu The IMU to use.
 */
class Orientation(val imu: BNO055IMU)
{
    // declare variables
    private val quat: Quaternion
        get() = imu.quaternionOrientation
    private val q0: Float
        get() = quat.w
    private val q1: Float
        get() = quat.x
    private val q2: Float
        get() = quat.y
    private val q3: Float
        get() = quat.z

    /** yaw (heading) */
    val psi: Double
        get() = atan2((2 * (q0 * q3 + q1 * q2)).toDouble(), (1 - 2 * (q2 * q2 + q3 * q3)).toDouble()) / Math.PI * 180

    /** pitch */
    val theta: Double
        get() = asin((2 * (q0 * q2 - q3 * q1)).toDouble()) / Math.PI * 180

    /** roll */
    val phi: Double
        get() = atan2((2 * (q0 * q1 + q2 * q3)).toDouble(), (1 - 2 * (q1 * q1 + q2 * q2)).toDouble()) / Math.PI * 180

    /** Gravity vector from the IMU */
    val gravity: Acceleration
        get() = imu.gravity

    /** Gravity X */
    val gx: Double
        get() = gravity.xAccel

    /** Gravity Y */
    val gy: Double
        get() = gravity.yAccel

    /** Gravity Z */
    val gz: Double
        get() = gravity.zAccel
}