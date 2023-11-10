package org.firstinspires.ftc.teamcode.subassemblies

import org.manchestermachinemakers.easyop.Subassembly
import org.manchestermachinemakers.easyop.Device
import com.qualcomm.robotcore.hardware.DcMotor

class DriveBase : Subassembly {
    @Device lateinit var leftFront: DcMotor
    @Device lateinit var rightFront: DcMotor
    @Device lateinit var leftRear: DcMotor
    @Device lateinit var rightRear: DcMotor
    
    /**
     * Taken from the RobotAutoDriveToAprilTagOmni example (2023)
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    fun moveRobot(x: Double, y: Double, yaw: Double) {
        // Calculate wheel powers.
        var leftFrontPower    =  x -y -yaw;
        var rightFrontPower   =  x +y +yaw;
        var leftBackPower     =  x +y -yaw;
        var rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        var max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftBackPower);
        rightRear.setPower(rightBackPower);
    }
}