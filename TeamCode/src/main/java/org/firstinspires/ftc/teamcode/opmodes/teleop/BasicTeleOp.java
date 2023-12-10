package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.contracts.Controllable;
import org.firstinspires.ftc.teamcode.subassemblies.Arm; // Imports Arm subassembly
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase; // Imports DriveBase subassembly
import org.manchestermachinemakers.easyop.Inject; // Imports Inject
import org.manchestermachinemakers.easyop.Linear; // Imports Linear

import kotlin.Pair;

@TeleOp(name = "Basic TeleOp")

// BasicTeleOp opmode which inherits from linear
public class BasicTeleOp extends Linear {
    @Inject public DriveBase driveBase; // Injects DriveBase code
    @Inject public Arm arm; // Injects Arm code (includes semi-auto and manual)

    // opLoop function
    @Override
    public void opLoop() { // Overrides opLoop function
        Controllable.Companion.runAll(
                new Pair<>(gamepad1, driveBase), // Associates first controller with drive base movement
                new Pair<>(gamepad2, arm)// Associates second controller with arm subassembly
        ); // End of runAll method
    } // End of opLoop function
} // End of BasicTeleOp class

// Note: The comments are to annoy the heck out of Aleks