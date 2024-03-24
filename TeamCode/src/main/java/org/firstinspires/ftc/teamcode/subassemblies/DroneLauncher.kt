package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log

class DroneLauncher(opMode: LinearOpMode): Subassembly(opMode, "Drone Launcher") {

    val launcher = ReleaseServo(hardwareMap.servo.get("airplane_launcher"), Pair(0.6, 0.72), Servo.Direction.REVERSE)
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
}