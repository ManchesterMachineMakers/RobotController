package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log

class PixelReleases(opMode: OpMode): Subassembly(opMode, "Pixel Droppers") {

    val left = ReleaseServo(
        hardwareMap.servo.get("left_release"),
        Pair(0.15, 0.40),
        Servo.Direction.FORWARD
    )
    val right = ReleaseServo(
        hardwareMap.servo.get("right_release"),
        Pair(0.4, 0.85),
        Servo.Direction.REVERSE
    )

    val telemetryData: Telemetry.Item = telemetry.addData("isOpen", "%b, %b", left.isOpen, right.isOpen)

    init {
        opMode.log("PixelReleases successfully initialized")
    }
    
    fun control(gamepad: Gamepad) {
        if (gamepad.left_bumper) left.open()
        else if (gamepad.left_trigger > 0.05) left.close()

        if (gamepad.right_bumper) right.open()
        else if (gamepad.right_trigger > 0.05) right.close()
    }
}