package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Names;

/**
 * Use this superclass to control a drive base with four mecanum wheels, one at each corner.
 */
public class MecanumDriveBase extends DriveBase {

    // @Device("left_front")
    public DcMotor motorLeftFront;

    // @Device("left_rear")
    public DcMotor motorLeftRear;

    // @Device("right_rear")
    public DcMotor motorRightRear;

    // @Device("right_front")
    public DcMotor motorRightFront;

    public MecanumDriveBase(HardwareMap hwMap) {
        super(hwMap);
    }

    @Override
    protected void initMotorConfigurations() {
        motorLeftFront  = hardwareMap.dcMotor.get(Names.motor_LeftFront);
        motorRightFront  = hardwareMap.dcMotor.get(Names.motor_RightFront);
        motorLeftRear  = hardwareMap.dcMotor.get(Names.motor_LeftRear);
        motorRightRear  = hardwareMap.dcMotor.get(Names.motor_RightRear);
        
        wheelMotors = new DcMotor[]{ 
            motorLeftFront, motorRightFront, 
            motorLeftRear, motorRightRear 
        };

        motorConfigurations.put(
            TravelDirection.strafeLeftForward, 
            new Direction[]{
                null, rightMotorDirection(Direction.FORWARD),
                leftMotorDirection(Direction.FORWARD), null
            });

        motorConfigurations.put(
            TravelDirection.strafeLeft, 
            new Direction[]{
                leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.FORWARD), 
                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE)
            });
        motorConfigurations.put(
            TravelDirection.strafeLeftBackward, 
            new Direction[]{
                leftMotorDirection(Direction.REVERSE), null,
                null, rightMotorDirection(Direction.REVERSE)
            });

        motorConfigurations.put(
            TravelDirection.strafeRightForward, 
            new Direction[]{
                leftMotorDirection(Direction.FORWARD), null, 
                null, rightMotorDirection(Direction.FORWARD)
            });
        motorConfigurations.put(
            TravelDirection.strafeRight, 
            new Direction[]{
                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE), 
                leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.FORWARD)
            });
        motorConfigurations.put(
            TravelDirection.strafeRightBackward, 
            new Direction[]{
                null, rightMotorDirection(Direction.REVERSE), 
                leftMotorDirection(Direction.REVERSE), null
            });

        motorConfigurations.put(
            TravelDirection.forward, 
            new Direction[]{
                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.FORWARD), 
                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.FORWARD)
            });
        motorConfigurations.put(
            TravelDirection.reverse, 
            new Direction[]{
                leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.REVERSE), 
                leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.REVERSE)
            });

        motorConfigurations.put(
            TravelDirection.pivotLeft, 
            new Direction[]{
                leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.FORWARD), 
                leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.FORWARD)
            });
        motorConfigurations.put(
            TravelDirection.pivotRight, 
            new Direction[]{
                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE), 
                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE)
            });
        motorConfigurations.put(
            TravelDirection.pitch,
            new Direction[]{
                null, null,
                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE)
            }
        );

        // super.initMotorConfigurations();
    }
    // for consistency, always use these methods to translate the actual motor direction
    // for the different sides of the bot.
    // if the motor orientation changes on the hardware, adjust these methods to suit.
    private static Direction rightMotorDirection(Direction externalDirection) {
        return externalDirection;
    }
    // reverse the motor direction for the motors on the right side of the bot.
    private static Direction leftMotorDirection(Direction externalDirection) {
        if (externalDirection == Direction.FORWARD) {
            return Direction.REVERSE;
        } else {
            return Direction.FORWARD;
        }
    }

}

