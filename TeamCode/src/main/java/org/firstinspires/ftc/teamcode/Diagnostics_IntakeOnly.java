package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.IntakeTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;

@TeleOp
public class Diagnostics_IntakeOnly extends DiagnosticsOpMode {

    @Override
    public Testable[] provides() {
        return new Testable[] {
                new ActiveIntake(hardwareMap, this)
        };
    }

}