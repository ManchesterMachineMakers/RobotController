package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.List;

/**
 * From IsaacArrayDemo 2/6/21
 */
public class Orientation {

    // declare variables
    BNO055IMU imu;

    Quaternion quat;
    float q0;
    float q1;
    float q2;
    float q3;
    public double psi; // yaw
    public double theta; // pitch
    public double phi; // roll
    public org.firstinspires.ftc.robotcore.external.navigation.Orientation angles;
    List<Axis> foo;
    public Acceleration gravity;

    //constructor
    public Orientation(BNO055IMU imu){
        this.imu = imu;
    }

    //methods
    void update(){
        quat = imu.getQuaternionOrientation();
        q0 = quat.w;
        q1 = quat.x;
        q2 = quat.y;
        q3 = quat.z;
        //
        // convert quaternion to Euler angles.
        // psi = yaw (or heading)
        psi = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) / Math.PI * 180;
        // theta = pitch
        theta = Math.asin(2 * (q0 * q2 - q3 * q1)) / Math.PI * 180;
        // phi = roll
        phi = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) / Math.PI * 180;
        // store gravity vector from IMU
        gravity = imu.getGravity();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }
    public double getPsi(){
        update();
        return psi; //yaw (heading)
    }
    double getTheta(){
        update();
        return theta; //pitch
    }
    double getPhi(){
        update();
        return phi; //roll
    }
    double getGx(){
        return gravity.xAccel;
    }
    double getGy(){
        return gravity.yAccel;
    }
    double getGz(){
        return gravity.zAccel;
    }
} //END:   class Orientation

