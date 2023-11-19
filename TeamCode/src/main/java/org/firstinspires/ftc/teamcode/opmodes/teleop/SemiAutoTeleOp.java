package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.SemiAutoArm;

@TeleOp(name = "Semi-Auto TeleOp", group = "arm")
public class SemiAutoTeleOp extends OpMode {

    Drivebase drivebase = new Drivebase();
    SemiAutoArm semiAutoArm = new SemiAutoArm();

    @Override
    public void init() {

        drivebase._gamepad = gamepad1;
        drivebase._telemetry = telemetry;
        drivebase._hardwareMap = hardwareMap;

        semiAutoArm._gamepad = gamepad2;
        semiAutoArm._telemetry = telemetry;
        semiAutoArm._hardwareMap = hardwareMap;

        drivebase.init();
        semiAutoArm.init();
    }

    @Override
    public void loop() {

        // prevents the robot from hurting itself
        if (semiAutoArm.needsStop) {
            requestOpModeStop();
        } else if (semiAutoArm.arm.getCurrent(CurrentUnit.AMPS) > 10) {
            terminateOpModeNow(); // in case previous statement fails
        }

        drivebase.loop();
        semiAutoArm.loop();

        drivebase.telemetry();
        semiAutoArm.telemetry();
    }
}
