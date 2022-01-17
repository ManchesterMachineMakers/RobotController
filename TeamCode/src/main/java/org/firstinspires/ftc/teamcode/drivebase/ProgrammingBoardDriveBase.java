package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Names;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

public class ProgrammingBoardDriveBase extends DriveBase {

    //@Device("left_front")
    public DcMotor motorLeftFront;

    public ProgrammingBoardDriveBase(OpMode opMode) {
        super(opMode.hardwareMap);
    }

    @Override
    protected void initMotorConfigurations() {
        motorLeftFront  = hardwareMap.get(DcMotor.class, RobotConfig.CURRENT.name("motor_LeftFront"));
        wheelMotors = new DcMotor[]{ motorLeftFront };

        motorConfigurations.put(TravelDirection.forward, new DcMotorSimple.Direction[]{ DcMotorSimple.Direction.FORWARD });
        motorConfigurations.put(TravelDirection.reverse, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});

        motorConfigurations.put(TravelDirection.pivotLeft, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});
        motorConfigurations.put(TravelDirection.pivotRight, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD});

        motorConfigurations.put(TravelDirection.strafeLeft, new DcMotorSimple.Direction[]{ DcMotorSimple.Direction.FORWARD });
        motorConfigurations.put(TravelDirection.strafeLeftBackward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});
        motorConfigurations.put(TravelDirection.strafeLeftForward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD});

        motorConfigurations.put(TravelDirection.strafeRight, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});
        motorConfigurations.put(TravelDirection.strafeRightBackward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD});
        motorConfigurations.put(TravelDirection.strafeRightForward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});
        motorConfigurations.put(TravelDirection.pitch, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD});
        // super.initMotorConfigurations();
    }
}
