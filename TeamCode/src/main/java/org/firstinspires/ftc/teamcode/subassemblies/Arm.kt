package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.util.GamepadManager
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

// Arm subassembly control
class Arm(opMode: OpMode) : Controllable, Subject {
    private val hardwareMap = opMode.hardwareMap
    val armMotor = hardwareMap.dcMotor.get("arm")
    val wrist = hardwareMap.servo.get("wrist")
    val rightRelease = hardwareMap.servo.get("right_release")
    val leftRelease = hardwareMap.servo.get("left_release")

    init {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER // Resets the encoder (distance tracking)
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    class PlacementInfo(
            val distToBase: Double,
            val alpha: Double,
            val beta: Double
    )

    fun getPlacementInfo(pixelRow: Int): PlacementInfo {
        val beta = asin(((d1 + pixelRow * d2 - l3) * sin(Math.PI / 3) + l2 * cos(Math.PI / 3) - h) / r)
        val alpha = Math.PI / 2 + Math.PI / 3 - gamma - beta
        val distToBase = r * cos(beta) + l2 * sin(Math.PI / 3) + cos(Math.PI / 3) * (l3 - d1 - pixelRow * d2)
        return PlacementInfo(distToBase, alpha, beta)
    }

    override fun controller(gamepad: GamepadManager) {
        armMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        armMotor.power = gamepad.gamepad.left_stick_y * 0.25
        //parallel.power = gamepad.gamepad.right_stick_y * 0.25
        if (gamepad.gamepad.left_bumper) {
            //leftDropper.power = -0.25
        } else if (gamepad.gamepad.left_trigger > 0.05) {
            //leftDropper.power = 0.25
        } else {
            //leftDropper.power = 0.0
        }
        if (gamepad.gamepad.right_bumper) {
            //rightDropper.power = 0.25
        } else if (gamepad.gamepad.right_trigger > 0.05) {
            //rightDropper.power = -0.25
        } else {
            //rightDropper.power = 0.0
        }
    }

    companion object {
        // Constants
        val gamma = atan2(16.0, 283.0)
        val l2 = 67.88
        val l3 = 59.56
        val r = 336.0
        val h = 287.75
        val d1 = 220.0
        val d2 = 88.9
    }
}
