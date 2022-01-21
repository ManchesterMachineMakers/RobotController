package org.firstinspires.ftc.teamcode.drivebase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Names;

@Deprecated // Ultimate Goal
public class InchwormMecanumDriveBase extends MecanumDriveBase {
    public Servo linearServo;

    public InchwormMecanumDriveBase(OpMode opMode) {
        super(opMode);
        linearServo = opMode.hardwareMap.servo.get(Names.servo_InchwormLifter);
    }

    @Override
    public void pitch(int degrees) {
        linearServo.setPosition((Math.sin(degrees) * 4.607) + 0.1);
    }
}
