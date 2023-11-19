package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.contracts.Controllable;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.manchestermachinemakers.easyop.Device;
import org.manchestermachinemakers.easyop.Subassembly;

// Arm subassembly control
public class Arm implements Subassembly, Controllable {
    // Initializes all the motors and servos
    @Device("motorEXP0")
    public DcMotor armMotor;
    @Device("servo0")
    public CRServo parallel;
    @Device("servo1")
    public CRServo rightDropper;
    @Device("servo2")
    public CRServo leftDropper;

    public static class PlacementInfo {
        public Double distToBase;
        public Double alpha;
        public Double beta;
    }
        // Constants
    static double gamma = Math.atan2(16.0, 283.0);
    static double l2 = 67.88;
    static double l3 = 59.56;
    static double r = 336.0;
    static double h = 287.75;
    static double d1 = 220.0;
    static double d2 = 88.9;

    @Override public void customInit(HardwareMap hardwareMap) {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Resets the encoder (distance tracking)
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public PlacementInfo getPlacementInfo(int pixelRow) {
        double localBeta = Math.asin(((((d1 + pixelRow * d2) - l3) * Math.sin(Math.PI / 3) + l2 * Math.cos(Math.PI / 3)) - h) / r);
        double localAlpha = Math.PI / 2 + Math.PI / 3 - gamma - localBeta;
        double localDistToBase =
                r * Math.cos(localBeta) + l2 * Math.sin(Math.PI / 3) + Math.cos(Math.PI / 3) * ((l3 - d1) - pixelRow * d2);
        return new PlacementInfo() {{
            this.distToBase = localDistToBase;
            this.alpha = localAlpha;
            this.beta = localBeta;
        }};
    }

    @Override
    public void controller(GamepadManager gamepad) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(gamepad.getGamepad().left_stick_y * 0.25);

        parallel.setPower(gamepad.getGamepad().right_stick_y * 0.25);

        if(gamepad.getGamepad().left_bumper) {
            leftDropper.setPower(-0.25);
        } else if(gamepad.getGamepad().left_trigger > 0.05) {
            leftDropper.setPower(0.25);
        } else {
            leftDropper.setPower(0);
        }

        if(gamepad.getGamepad().right_bumper) {
            rightDropper.setPower(0.25);
        } else if(gamepad.getGamepad().right_trigger > 0.05) {
            rightDropper.setPower(-0.25);
        } else {
            rightDropper.setPower(0);
        }
    }
}
