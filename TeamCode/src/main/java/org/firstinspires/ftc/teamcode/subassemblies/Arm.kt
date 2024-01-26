package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.teamcode.autonomous.path.motorEncoderEventsPerMM
import org.firstinspires.ftc.teamcode.autonomous.path.motorEncoderEventsPerRevolution
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.teamcode.util.bases.BaseArm
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

fun Servo.release() {

}

// Arm subassembly control
class Arm(opMode: OpMode) : Controllable, Subject {
    private val hardwareMap = opMode.hardwareMap
    val armMotor = hardwareMap.dcMotor.get("arm")
    val wrist = hardwareMap.servo.get("wrist")
    val rightRelease = hardwareMap.servo.get("right_release")
    val leftRelease = hardwareMap.servo.get("left_release")
    val touchSensor = hardwareMap.touchSensor.get("intake")

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

    // TODO: run the drivebase in the right direction
    fun placePixel(driveBase: DriveBase, placementInfo: PlacementInfo) {
        // math from matlab (armcode.mlx)
        val ticksToBase = placementInfo.distToBase * motorEncoderEventsPerMM

        val servoDegreesMax = 300.0
        val servoDegrees = clamp(((placementInfo.alpha - PI/2) / (2*PI)) * 360, 0.0, servoDegreesMax)
        val motorTicks = (placementInfo.beta / (2*PI)) * motorEncoderEventsPerRevolution

        armMotor.targetPosition = motorTicks.roundToInt()
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        wrist.position = servoDegrees

        driveBase.runToPosition(ticksToBase.roundToInt())

    }

    data class DropCorrection(val armPosition: Int, val wristPosition: Double)

    enum class RelativeDropTarget {
        easel, floor
    }

    fun relativeWristPosition(target: RelativeDropTarget): Double {
        val theta = when(target) {
            RelativeDropTarget.easel -> 60
            RelativeDropTarget.floor -> 120
        }
        val armAngle = 360 * armMotor.currentPosition / BaseArm.ARM_ENCODER_RES
        val servoAngle = 90 + theta - CtSemiAutoArm.GAMMA - armAngle // degrees?
        return (servoAngle - 90) / (0.53 * 300) - 0.5 * 0.53 // from degrees? to servo range
    }

    fun drop(): DropCorrection {
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        val initialWristPos = wrist.position
        while(!touchSensor.isPressed) {
            armMotor.power = -0.2
            wrist.position--
        }
        armMotor.power = 0.0

        return DropCorrection(armMotor.currentPosition, initialWristPos)
    }

    fun raise(correction: DropCorrection) {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armMotor.targetPosition = -correction.armPosition
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        armMotor.power = 0.2
        wrist.position = correction.wristPosition
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
