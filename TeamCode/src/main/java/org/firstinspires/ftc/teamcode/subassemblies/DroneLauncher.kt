package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.Subassembly

class DroneLauncher(opMode: OpMode): Subassembly(opMode, "Drone Launcher") {

    val launcher = ReleaseServo(hardwareMap.servo.get("airplane_launcher"))
    var buttonWasPressed = false

    init {
        launcher.scaleRange = Pair(0.7, 0.78)
        launcher.direction = Servo.Direction.REVERSE

        telemetry.addData(">", "Drone Launcher Subassembly Ready")
        telemetry.update()
    }
    
    fun control(button: Boolean) { // I need to figure out GamepadManager, but this'll have to do.
        if (button && !buttonWasPressed) {
            buttonWasPressed = true
            launcher.toggle()
        }
        else if (!button) buttonWasPressed = false
    }
}