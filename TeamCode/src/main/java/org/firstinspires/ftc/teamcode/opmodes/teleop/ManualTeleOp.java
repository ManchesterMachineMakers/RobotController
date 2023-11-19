package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.ManualArm;

@TeleOp(name = "Manual Arm TeleOp", group = "arm")
public class ManualTeleOp extends OpMode {

    Drivebase drivebase = new Drivebase();
    ManualArm manualArm = new ManualArm();

    @Override
    public void init() {

        drivebase._gamepad = gamepad1;
        drivebase._telemetry = telemetry;
        drivebase._hardwareMap = hardwareMap;

        manualArm._gamepad = gamepad2;
        manualArm._telemetry = telemetry;
        manualArm._hardwareMap = hardwareMap;

        drivebase.init();
        manualArm.init();
    }

    @Override
    public void loop() {

        // overcurrent protection
        if (manualArm.needsStop) {
            requestOpModeStop();
        } else if (manualArm.arm.getCurrent(CurrentUnit.AMPS) > 10) {
            terminateOpModeNow();
        }

        drivebase.loop();
        manualArm.loop();

        drivebase.telemetry();
        manualArm.telemetry();
    }
}
