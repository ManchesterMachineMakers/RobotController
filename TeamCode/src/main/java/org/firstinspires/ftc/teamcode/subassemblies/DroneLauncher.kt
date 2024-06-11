package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.ServoDirections
import org.firstinspires.ftc.teamcode.util.ServoRanges
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log

class DroneLauncher(opMode: LinearOpMode): Subassembly(opMode, "Drone Launcher") {

    val launcher = ReleaseServo(
        hardwareMap.servo.get("airplane_launcher"),
        ServoRanges.DRONE_LAUNCHER,
        ServoDirections.DRONE_LAUNCHER
    )
    var buttonWasPressed = false

    init {
        opMode.log("DroneLauncher successfully initialized")
    }
    
    fun control(button: Boolean) { // I need to figure out GamepadManager, but this'll have to do.
        if (button && !buttonWasPressed) {
            buttonWasPressed = true
            launcher.toggle()
        }
        else if (!button) buttonWasPressed = false
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addData("isOpen", launcher.isOpen)
    }
}