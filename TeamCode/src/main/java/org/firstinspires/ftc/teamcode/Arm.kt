package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion

object Arm : CustomBlocksOpModeCompanion() {
    var longArm: DcMotor? = null
    var shortArm: Servo? = null
    var rotationalBase: Servo? = null
    fun initHardware() {
        longArm = hardwareMap.dcMotor.get("longarm")
        shortArm = hardwareMap.servo.get("shortarm")
        rotationalBase = hardwareMap.servo.get("rotationalbase")
        longArm?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun zero() {
        shortArm?.position = 0.0
        rotationalBase?.position = 0.0
    }

    fun controller() {
        if(shortArm == null || longArm == null || rotationalBase == null) return
        val currentServoPos = shortArm!!.position;
        if (gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0) {
            longArm!!.power = gamepad2.right_stick_y / 40.0
        } else {
            if (gamepad2.right_stick_y == 0f) {
                longArm!!.power = 0.0;
            }
        }
        if (gamepad2.left_stick_y > 0) {
            shortArm!!.position = 0.0025 + currentServoPos;
        }
        if (gamepad2.left_stick_y < 0) {
            shortArm!!.position = -0.0025 + currentServoPos;
        }
        if (gamepad2.left_stick_y == 0f) {
            shortArm!!.position = currentServoPos;
        }

        if (gamepad2.right_bumper) {
            rotationalBase!!.position += 0.0005
        } else if (gamepad2.left_bumper) {
            rotationalBase!!.position -= 0.0005
        }
    }
}