package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.*
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.manchestermachinemakers.easyop.Device
import org.manchestermachinemakers.easyop.Subassembly

// Arm subassembly control
class Arm : Subassembly, Controllable {
    // Initializes all the motors and servos
    @Device("motorEXP0") lateinit var armMotor: DcMotor
    @Device("servo0") lateinit var parallel: Servo
    @Device("servo1") lateinit var rightDropper: Servo
    @Device("servo2") lateinit var leftDropper: Servo

    data class PlacementInfo(val distToBase: Double, val alpha: Double, val beta: Double)

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

    override fun customInit(hardwareMap: HardwareMap) {
        armMotor.mode =
                DcMotor.RunMode.STOP_AND_RESET_ENCODER // Resets the encoder (distance tracking)
        armMotor.mode =
                DcMotor.RunMode.RUN_TO_POSITION // Sets the motor to run to specific positions
    }

    fun getPlacementInfo(pixelRow: Int): PlacementInfo {
        val beta = asin(((((d1 + pixelRow * d2) - l3) * sin(PI / 3) + l2 * cos(PI / 3)) - h) / r)
        val alpha = PI / 2 + PI / 3 - gamma - beta
        val distToBase =
                r * cos(beta) + l2 * sin(PI / 3) + cos(PI / 3) * ((l3 - d1) - pixelRow * d2)
        return PlacementInfo(distToBase, alpha, beta)
    }

    override fun controller(gamepad: GamepadManager) {
        // Left bumper releases left dropper/brush
        gamepad.on("left_bumper") {
            leftDropper.position = 90.0 / 360 // One servo must move in opposite direction
        }
        gamepad.off(listOf("left_bumper")) { leftDropper.position = 0.0 }

        // Right bumber releases right dropper/brush
        gamepad.on("right_bumper") {
            rightDropper.position = -90.0 / 360 // One servo must move in opposite direction
        }
        gamepad.off(listOf("right_bumper")) { rightDropper.position = 0.0 }

        armMotor.targetPosition += (gamepad.gamepad.left_stick_y * 10).roundToInt()
        parallel.position += gamepad.gamepad.right_stick_y * 0.1
    }
}
