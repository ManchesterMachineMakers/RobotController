package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.ManualArm;

@TeleOp(name = "Manual Arm TeleOp")
public class ManualTeleOp extends OpMode {

    Drivebase drivebase = new Drivebase();
    ManualArm manualArm = new ManualArm();

    @Override
    public void init() {

        drivebase._Gamepad = gamepad1;
        drivebase._Telemetry = telemetry;
        drivebase._HardwareMap = hardwareMap;

        manualArm._Gamepad = gamepad2;
        manualArm._Telemetry = telemetry;
        manualArm._HardwareMap = hardwareMap;

        drivebase.init();
        manualArm.init();
    }

    @Override
    public void loop() {

        drivebase.loop();
        manualArm.loop();

        drivebase.telemetry();
        manualArm.telemetry();
    }
}
