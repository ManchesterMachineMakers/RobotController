package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.util.Names;
import org.firstinspires.ftc.teamcode.util.Subassembly;

/**
 * Delivery mechanism for Freight Frenzy
 * Drawer slide to extend to the proper height
 * Grabber to keep or release the freight.
 *
 */
public class Delivery implements Subassembly {

    public Delivery(OpMode opMode) {
        deliveryServo = opMode.hardwareMap.servo.get(Names.servo_DeliverySlide);
        grabberServo = opMode.hardwareMap.servo.get(Names.servo_GrabberClaw);
    }

    public Servo deliveryServo;
    public Servo grabberServo;

    public void down() {
        deliveryServo.setPosition(0.2);
    }
    public void deliverLow() { deliveryServo.setPosition(0.35); }
    public void deliverMid() { deliveryServo.setPosition(0.4); }
    public void deliverHigh() { deliveryServo.setPosition(0.5); }
    public void up() {
        deliveryServo.setPosition(0.55);
    }
    public void rest() { deliveryServo.setPosition(0.75);}
    public void grab() {
      grabberServo.setPosition(0.8);
    }
    public void release() {
      grabberServo.setPosition(0.6);
    }
    /**
    * Down, grab, up.
    */
    public void grabWrapper() {
      down();
      grab();
      up();
    }
    /**
    * Down, release, up.
    */
    public void releaseWrapper() {
      down();
      release();
      up();
    }
}
