package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.ExportToBlocks

object Arm : CustomBlocksOpModeCompanion() {
    var longArm: DcMotor? = null
    var shortArm: Servo? = null
    var rotationalBase: Servo? = null

    override fun exists() = hardwareMap.dcMotor.contains("longarm") && hardwareMap.servo.contains("shortarm") && hardwareMap.dcMotor.contains("rotationalbase")

    override fun initHardware() {
        longArm = hardwareMap.dcMotor.get("longarm")
        shortArm = hardwareMap.servo.get("shortarm")
        rotationalBase = hardwareMap.servo.get("rotationalbase")
        longArm?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        longArm?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    @JvmStatic @ExportToBlocks
    fun zero() {
        shortArm?.position = 0.0
        rotationalBase?.position = 0.0
    }

    @JvmStatic @ExportToBlocks
    fun controller() {
        if(shortArm == null || longArm == null || rotationalBase == null) return
        val currentServoPos = shortArm!!.position;
        if (gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0) {
            longArm!!.power = gamepad2.right_stick_y / 3.0
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