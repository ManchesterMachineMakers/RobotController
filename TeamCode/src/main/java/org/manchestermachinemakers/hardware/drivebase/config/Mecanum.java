package org.manchestermachinemakers.hardware.drivebase.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.manchestermachinemakers.hardware.drivebase.DriveBase;

public class Mecanum extends DriveBase.Configuration {
    @Override
    public void init() {
        setMotors(
                "left_front",                    "right_front",
                "left_rear",                     "right_rear"
        );

        put(DriveBase.TravelDirection.base, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);
        put(DriveBase.TravelDirection.forward, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        put(DriveBase.TravelDirection.reverse, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);
        put(DriveBase.TravelDirection.pivotLeft, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        put(DriveBase.TravelDirection.pivotRight, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        put(DriveBase.TravelDirection.strafeLeft, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        put(DriveBase.TravelDirection.strafeLeftForward, null, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, null);
        put(DriveBase.TravelDirection.strafeLeftBackward, DcMotorSimple.Direction.REVERSE, null, null, DcMotorSimple.Direction.REVERSE);
        put(DriveBase.TravelDirection.strafeRight, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        put(DriveBase.TravelDirection.strafeRightForward, DcMotorSimple.Direction.FORWARD, null, null, DcMotorSimple.Direction.FORWARD);
        put(DriveBase.TravelDirection.strafeRightBackward, null, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, null);
        put(DriveBase.TravelDirection.pitch, null, null, null, null);
    }
}
