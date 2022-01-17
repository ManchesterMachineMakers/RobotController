package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Names;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

public class IMUSensor {
    private BNO055IMU imu;
    private HardwareMap hardwareMap;

    /**
     * Pass in the hardware map from the opmode.
     * @param hwMap The hardware map.
     */
    public IMUSensor(HardwareMap hwMap) {
        hardwareMap = hwMap;
    }

    /**
     * Initialize the IMU with standard parameters.
     */
    public BNO055IMU initIMU() {
        //declare variables for quaternion math using BNO055 IMU
        // Create new IMU Parameters object.
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        //  set mode to "IMU". This will use the
        // device's gyro and its accelerometer
        // to calculate the relative orientation of the
        // hub and therefore the robot.
        // Gyro gets calibrated automatically in this mode.
        imuParameters.mode = BNO055IMU.SensorMode.NDOF;
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Set acceleration integration algorithm to
        // "naive" which will give velocity & position
        imuParameters.accelerationIntegrationAlgorithm = null;
        // Enable logging.
        imuParameters.loggingEnabled = false;

        // Initialize IMU.
        return initIMU(imuParameters);
    }

    /**
     * Initialize the IMU with custom parameters.
     * @param imuParameters
     */
    public BNO055IMU initIMU(BNO055IMU.Parameters imuParameters) {
        imu = hardwareMap.get(BNO055IMU.class, RobotConfig.CURRENT.name("imu"));
        imu.initialize(imuParameters);
        RobotLog.i("IMU initialized.");
        return imu;
    }

}
