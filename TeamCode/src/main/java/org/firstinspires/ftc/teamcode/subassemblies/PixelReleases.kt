package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.Subassembly

class PixelReleases(opMode: OpMode): Subassembly(opMode, "Pixel Droppers") {

    val leftReleaseServo = opMode.hardwareMap.servo.get("left_release")
    val rightReleaseServo = opMode.hardwareMap.servo.get("right_release")

    val leftRelease = ReleaseServo(leftReleaseServo)
    val rightRelease = ReleaseServo(rightReleaseServo)

    init {
        leftReleaseServo.scaleRange(0.15, 0.40) // 22.5% of 300-degree range
        leftReleaseServo.direction = Servo.Direction.FORWARD

        rightReleaseServo.scaleRange(0.6, 1.0) // 22.5% of 300-degree range
        rightReleaseServo.direction = Servo.Direction.REVERSE

        telemetry.addData(">", "Pixel Droppers Subassembly Ready")
    }

    fun control(gamepad: Gamepad) {
        if (gamepad.left_bumper) leftRelease.open()
        else if (gamepad.left_trigger > 0.05) leftRelease.close()

        if (gamepad.right_bumper) rightRelease.open()
        else if (gamepad.right_trigger > 0.05) rightRelease.close()
    }
}