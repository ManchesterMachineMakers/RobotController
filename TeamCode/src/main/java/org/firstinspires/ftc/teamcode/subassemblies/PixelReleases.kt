package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.ServoDirections
import org.firstinspires.ftc.teamcode.util.ServoRanges
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log

class PixelReleases(opMode: LinearOpMode): Subassembly(opMode, "Pixel Droppers") {

    val left = ReleaseServo(
        hardwareMap.servo.get("left_release"),
        ServoRanges.LEFT_RELEASE,
        ServoDirections.LEFT_RELEASE
    )
    val right = ReleaseServo(
        hardwareMap.servo.get("right_release"),
        ServoRanges.RIGHT_RELEASE,
        ServoDirections.RIGHT_RELEASE
    )

    init {
        opMode.log("PixelReleases successfully initialized")
    }
    
    fun control(gamepad: Gamepad) {
        if (gamepad.left_bumper) left.open()
        else if (gamepad.left_trigger > 0.05) left.close()

        if (gamepad.right_bumper) right.open()
        else if (gamepad.right_trigger > 0.05) right.close()
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addData("Left", left.isOpen)
        telemetry.addData("Right", right.isOpen)
    }
}