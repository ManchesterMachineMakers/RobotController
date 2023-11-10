package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.subassemblies.Arm // Imports Arm subassembly
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase // Imports DriveBase subassembly
import org.manchestermachinemakers.easyop.Inject // Imports Inject
import org.manchestermachinemakers.easyop.Linear // Imports Linear

@TeleOp(name = "Basic TeleOp")

// BasicTeleOp function which inherits from linear
class BasicTeleOp: Linear() {
    @Inject lateinit var driveBase: DriveBase // Injects DriveBase code
    @Inject lateinit var arm: Arm // Injects Arm code (includes semi-auto and manual)

    // opLoop function
    override fun opLoop() { // Overrides opLoop function
        Controllable.runAll(
                gamepad1 to driveBase, // Associates first controller with drive base movement
                gamepad2 to arm // Associates second controller with arm subassembly
        ) // End of runAll method
    } // End of opLoop function
} // End of BasicTeleOp class

// Note: The comments are to annoy the heck out of Aleks