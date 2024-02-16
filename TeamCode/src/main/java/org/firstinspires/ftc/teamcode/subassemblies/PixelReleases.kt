package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.ReleaseServo
import org.firstinspires.ftc.teamcode.util.Subassembly

class PixelReleases(opMode: OpMode): Subassembly(opMode, "Pixel Droppers") {

    val left = ReleaseServo(hardwareMap.servo.get("left_release"))
    val right = ReleaseServo(hardwareMap.servo.get("right_release"))

    init {
        left.scaleRange = Pair(0.15, 0.40) // 22.5% of 300-degree range
        left.direction = Servo.Direction.FORWARD

        right.scaleRange = Pair(0.6, 1.0) // 22.5% of 300-degree range
        right.direction = Servo.Direction.REVERSE

        telemetry.addData(">", "Pixel Droppers Subassembly Ready")
        telemetry.update()
    }
}