package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.teamcode.util.RobotConfig
import org.firstinspires.ftc.teamcode.util.Subassembly

class IMUManager(private val opMode: LinearOpMode) : Subassembly {
    val imu: BNO055IMU = opMode.hardwareMap[BNO055IMU::class.java, "imu"]
    private val imuParameters = BNO055IMU.Parameters()
    val orientation = Orientation(imu)
    init {
        //  set mode to "IMU". This will use the
        // device's gyro and its accelerometer
        // to calculate the relative orientation of the
        // hub and therefore the robot.
        // Gyro gets calibrated automatically in this mode.
        imuParameters.mode = BNO055IMU.SensorMode.NDOF
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        // Set acceleration integration algorithm to
        // "naive" which will give velocity & position
        imuParameters.accelerationIntegrationAlgorithm = null
        // Enable logging.
        imuParameters.loggingEnabled = false
        // Initialize IMU.
        imu.initialize(imuParameters)
    }

    fun getRobotLocation(): OpenGLMatrix? {
        //TODO: implement
        return null
    }
}