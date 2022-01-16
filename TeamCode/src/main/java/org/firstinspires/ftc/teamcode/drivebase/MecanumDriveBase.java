package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    public MecanumDriveBase(OpMode opMode) {
        super(opMode.hardwareMap);
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

        // We use the motorConfigurations to determine whether the power and encoder
        // values should be positive or negative when attempting any particular direction.
        // The directions are typically used in Autonomous programming.
        // TeleOp should use the TravelDirection.forward configuration.
        motorConfigurations.put(TravelDirection.base,
                new Direction[]{
                        Direction.FORWARD, Direction.FORWARD,
                        Direction.REVERSE, Direction.REVERSE
                });
        motorConfigurations.put(
                TravelDirection.forward,
                new Direction[]{
                        Direction.FORWARD, Direction.FORWARD,
                        Direction.FORWARD, Direction.FORWARD
                });
        motorConfigurations.put(
                TravelDirection.reverse,
                new Direction[]{
                        Direction.REVERSE, Direction.REVERSE,
                        Direction.REVERSE, Direction.REVERSE
                });

        motorConfigurations.put(
                TravelDirection.pivotLeft,
                new Direction[]{
                        Direction.REVERSE, Direction.FORWARD,
                        Direction.REVERSE, Direction.FORWARD
                });
        motorConfigurations.put(
                TravelDirection.pivotRight,
                new Direction[]{
                        Direction.FORWARD, Direction.REVERSE,
                        Direction.FORWARD, Direction.REVERSE
                });
        motorConfigurations.put(
            TravelDirection.strafeLeftForward,
            new Direction[]{
                null, Direction.FORWARD,
                Direction.FORWARD, null
            });

        motorConfigurations.put(
            TravelDirection.strafeLeft,
            new Direction[]{
                    Direction.REVERSE, Direction.FORWARD,
                    Direction.FORWARD, Direction.REVERSE
            });
        motorConfigurations.put(
            TravelDirection.strafeLeftBackward,
            new Direction[]{
                Direction.REVERSE, null,
                null, Direction.REVERSE
            });

        motorConfigurations.put(
            TravelDirection.strafeRightForward,
            new Direction[]{
                Direction.FORWARD, null,
                null, Direction.FORWARD
            });
        motorConfigurations.put(
            TravelDirection.strafeRight,
            new Direction[]{
                    Direction.FORWARD, Direction.REVERSE,
                    Direction.REVERSE, Direction.FORWARD
            });
        motorConfigurations.put(
            TravelDirection.strafeRightBackward,
            new Direction[]{
                null, Direction.REVERSE,
                Direction.REVERSE, null
            });
//        motorConfigurations.put(
//            TravelDirection.pitch,
//            new Direction[]{
//                null, null,
//                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE)
//            }
//        );

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
        return externalDirection;
//        if (externalDirection == Direction.FORWARD) {
//            return Direction.REVERSE;
//        } else {
//            return Direction.FORWARD;
//        }
    }

}

