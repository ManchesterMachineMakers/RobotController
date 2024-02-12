package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.autonomous.path.motorEncoderEventsPerRevolution
import org.firstinspires.ftc.teamcode.contracts.Controllable
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.CtSemiAutoArm
import org.firstinspires.ftc.teamcode.subassemblies.miles.arm.DoNotBreakThisArm
import org.firstinspires.ftc.teamcode.util.GamepadManager
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

fun Servo.open() { position = 1.0 }
fun Servo.close() { position = 0.0 }

// Arm subassembly control
class Arm(private val opMode: OpMode) : Controllable, Subject {
    private val hardwareMap = opMode.hardwareMap
    val armMotor = hardwareMap.dcMotor.get("arm") as DcMotorEx
    val wrist = hardwareMap.servo.get("wrist")
    val rightRelease = hardwareMap.servo.get("right_release")
    val leftRelease = hardwareMap.servo.get("left_release")
    val touchSensor = hardwareMap.touchSensor.get("intake")

    init {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER // Resets the encoder (distance tracking)
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Wrist servo configuration
        wrist.scaleRange(0.25, 0.78) // 53% of 300-degree range
        wrist.direction = Servo.Direction.FORWARD
    }


    class PlacementInfo(
            val distToBase: Double,
            val alpha: Double,
            val beta: Double
    )

    fun getPlacementInfo(pixelRow: Int): PlacementInfo {
        val beta = asin(((D1 + pixelRow * D2 - L3) * sin(Math.PI / 3) + L2 * cos(Math.PI / 3) - H) / R)
        val alpha = Math.PI / 2 + Math.PI / 3 - GAMMA - beta
        val distToBase = R * cos(beta) + L2 * sin(Math.PI / 3) + cos(Math.PI / 3) * (L3 - D1 - pixelRow * D2)
        return PlacementInfo(distToBase, alpha, beta)
    }

    fun placePixel(placementInfo: PlacementInfo) {
        // math from matlab (armcode.mlx)

        val servoDegreesMax = 300.0
        val servoDegrees = clamp(((placementInfo.alpha - PI/2) / (2*PI)) * 360, 0.0, servoDegreesMax)
        val motorTicks = (placementInfo.beta / (2*PI)) * motorEncoderEventsPerRevolution

        armMotor.targetPosition = motorTicks.roundToInt()
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        wrist.position = servoDegrees
    }

    data class DropCorrection(val armPosition: Int, val wristPosition: Double)

    enum class RelativeDropTarget {
        easel, floor
    }

    fun relativeWristPosition(target: RelativeDropTarget): Double {
        val theta = when(target) {
            RelativeDropTarget.easel -> 60
            RelativeDropTarget.floor -> 120 // 0.25 servo position (or 130?)
        }
        val armAngle = 360 * armMotor.currentPosition / ARM_ENCODER_RES
        val servoAngle = 90 + theta - CtSemiAutoArm.GAMMA - armAngle // degrees
        return (servoAngle - 90) / (0.53 * 300) - 0.5 * 0.53 // from degrees to servo range
    }

    fun drop() {
        while(!touchSensor.isPressed && armMotor.isBusy) {
            armMotor.power = -0.2
            opMode.telemetry.addData("Touch Sensor", touchSensor.isPressed)
            opMode.telemetry.addData("Arm Motor", armMotor.currentPosition)
            opMode.telemetry.addData("Wrist Servo", wrist.position)
            opMode.telemetry.update()
        }
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armMotor.power = 0.0
        // go again, zeroed so the wrist position calculates correctly
        opMode.telemetry.addLine("Zeroed the Arm Motor")
        opMode.telemetry.log()
        Thread.sleep(10)

        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        while(!touchSensor.isPressed) {
            armMotor.power = -0.2
            wrist.position = relativeWristPosition(RelativeDropTarget.floor)
            Thread.sleep(10)
        }
        armMotor.power = 0.0
        opMode.telemetry.update()
    }

    fun raise() {
        armMotor.targetPosition = 200
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        armMotor.power = 0.2
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

    fun telemetry() {
        val telemetry = opMode.telemetry
        telemetry.addData("Touch Sensor", touchSensor.isPressed)
        telemetry.addData("Arm Motor", armMotor.currentPosition)
        telemetry.addData("Wrist Servo", wrist.position)
        telemetry.update()
    }

    fun startOvercurrentProtection() {
        Thread {
            while(true) {
                if (armMotor.isOverCurrent) {
                    if (armMotor.getCurrent(CurrentUnit.AMPS) > DoNotBreakThisArm.ARM_OVERCURRENT_THRESHOLD * 1.4) {
                        opMode.requestOpModeStop()
                    } else {
                        armMotor.targetPosition = 0
                        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    }
                }
                Thread.sleep(20)
            }
        }.start()
    }

    val overcurrentProtection = Thread {
        while(true) {
            if (armMotor.isOverCurrent) {
                if (armMotor.getCurrent(CurrentUnit.AMPS) > DoNotBreakThisArm.ARM_OVERCURRENT_THRESHOLD * 1.4) {
                    opMode.requestOpModeStop()
                } else {
                    armMotor.targetPosition = 0
                    armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                }
            }
            Thread.sleep(20)
        }
    }

    companion object { // constants
        const val ARM_ENCODER_RES = 2786.2 * 2 // PPR of motor * 2:1 gearing ratio
        // math
        val GAMMA = atan2(16.0, 283.0)
        const val L2 = 67.88
        const val L3 = 59.56
        const val R = 336.0
        const val H = 287.75
        const val D1 = 220.0
        const val D2 = 88.9
    }
}
