package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class Location extends Destination {

    public Orientation orientation;

    public Location(Destination position, Orientation orientation) {
        this.destName = position.destName;
        this.setX(position.getX());
        this.setY(position.getY());
        this.orientation = orientation;
    }
    public Location(BNO055IMU imu, String name, float destX, float destY, float roll, float pitch, float yaw){
        orientation = new Orientation(imu);
        orientation.psi = yaw;
        orientation.theta = pitch;
        orientation.phi = roll;
        this.destName = name;
        this.setX(destX);
        this.setY(destY);
    }
}
