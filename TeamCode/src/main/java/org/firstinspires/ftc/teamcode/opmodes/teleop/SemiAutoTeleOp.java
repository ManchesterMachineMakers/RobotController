package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subassemblies.miles.Drivebase;
import org.firstinspires.ftc.teamcode.subassemblies.miles.SemiAutoArm;

@TeleOp(name = "Full Semi-Auto TeleOp")
public class SemiAutoTeleOp extends OpMode {

    Drivebase drivebase = new Drivebase();
    SemiAutoArm semiAutoArm = new SemiAutoArm();

    @Override
    public void init() {

        drivebase._Gamepad = gamepad1;
        drivebase._Telemetry = telemetry;
        drivebase._HardwareMap = hardwareMap;

        semiAutoArm._Gamepad = gamepad2;
        semiAutoArm._Telemetry = telemetry;
        semiAutoArm._HardwareMap = hardwareMap;

        drivebase.init();
        semiAutoArm.init();
    }

    @Override
    public void loop() {

        drivebase.loop();
        semiAutoArm.loop();

        drivebase.telemetry();
        semiAutoArm.telemetry();
    }
}
