package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.Subassembly

class DroneLauncher(opMode: OpMode): Subassembly(opMode, "Drone Launcher") {

    val launcher = ReleaseServo(hardwareMap.servo.get("airplane_launcher"))
    private var registerButton = false

    init {
        launcher.scaleRange = Pair(0.7, 0.78)
        launcher.direction = Servo.Direction.REVERSE

        telemetry.addData(">", "Drone Launcher Subassembly Ready")
    }

    fun control(button: Boolean) {
        if (button && !registerButton) {
            registerButton = true
            launcher.toggle()
        }
        else if (!button) registerButton = false
    }
}