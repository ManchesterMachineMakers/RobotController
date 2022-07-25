package org.manchestermachinemakers.hardware.drivebase.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.manchestermachinemakers.hardware.drivebase.DriveBase.TravelDirection;

import org.manchestermachinemakers.hardware.drivebase.DriveBase;

import java.util.HashMap;

public class ProgrammingBoard extends DriveBase.Configuration {
    @Override
    public void init() {
        setMotors("left_front");
        put(TravelDirection.forward, DcMotorSimple.Direction.FORWARD);
        put(TravelDirection.reverse, DcMotorSimple.Direction.REVERSE);
        put(TravelDirection.pivotLeft, DcMotorSimple.Direction.REVERSE);
        put(TravelDirection.pivotRight, DcMotorSimple.Direction.FORWARD);
        put(TravelDirection.strafeLeft, DcMotorSimple.Direction.FORWARD);
        put(TravelDirection.strafeLeftBackward, DcMotorSimple.Direction.REVERSE);
        put(TravelDirection.strafeLeftForward, DcMotorSimple.Direction.FORWARD);
        put(TravelDirection.strafeRight, DcMotorSimple.Direction.REVERSE);
        put(TravelDirection.strafeRightBackward, DcMotorSimple.Direction.FORWARD);
        put(TravelDirection.strafeRightForward, DcMotorSimple.Direction.REVERSE);
        put(TravelDirection.pitch, DcMotorSimple.Direction.FORWARD);
    }
}
