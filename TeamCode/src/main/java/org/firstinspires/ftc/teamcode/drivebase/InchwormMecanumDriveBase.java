package org.firstinspires.ftc.teamcode.drivebase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Names;

public class InchwormMecanumDriveBase extends MecanumDriveBase {
    public Servo linearServo;

    public InchwormMecanumDriveBase(HardwareMap hwMap) {
        super(hwMap);
        linearServo = hwMap.servo.get(Names.servo_InchwormLifter);
    }

    @Override
    public void pitch(int degrees) {
        linearServo.setPosition((Math.sin(degrees) * 4.607) + 0.1);
    }
}
