package org.firstinspires.ftc.teamcode.subassemblies

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.autonomous.path.motorEncoderEventsPerRevolution
import org.firstinspires.ftc.teamcode.util.OvercurrentProtection
import org.firstinspires.ftc.teamcode.util.ServoRanges
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.clamp
import org.firstinspires.ftc.teamcode.util.degreesToServoPosition
import org.firstinspires.ftc.teamcode.util.encoderPositionToDegrees
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.powerCurve
import org.firstinspires.ftc.teamcode.util.toDegrees
import org.firstinspires.ftc.teamcode.util.toRadians
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

// Arm subassembly control
class Arm(opMode: LinearOpMode) : Subject, Subassembly(opMode, "Arm") {
    val armMotor = hardwareMap.dcMotor.get("arm") as DcMotorEx
    val distanceSensor = hardwareMap.get(DistanceSensor::class.java, "intake_distance")
    val wrist = hardwareMap.servo.get("wrist")
    var wristAlignment: WristAlignment? = WristAlignment.EASEL
    var wristOffset = 0.0 // in degrees
    var dpadWasUsed = false

    init {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER // Resets the encoder (distance tracking)
        Thread.sleep(10)
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Wrist servo configuration
        wrist.scaleRange(ServoRanges.WRIST.first, ServoRanges.WRIST.second) // 53% of 300-degree range
        wrist.direction = Servo.Direction.REVERSE

        opMode.log("Arm successfully initialized")
    }


    class PlacementInfo(
            val distToBase: Double,
            val alpha: Double,
            val beta: Double
    )

    data class DropCorrection(val armPosition: Int, val wristPosition: Double)

    enum class WristAlignment {
        EASEL, FLOOR
    }

    val overcurrentProtection = OvercurrentProtection(armMotor, 5.0, {
        armMotor.targetPosition = 0
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }, { armMotor.setMotorDisable() })

    fun control(gamepad: Gamepad) {
        if (!armMotor.isOverCurrent) // lock out controls if overcurrent
            armMotor.power = powerCurve(-gamepad.left_stick_y.toDouble())

        armMotor.mode = // arm calibration
            if (gamepad.b) DcMotor.RunMode.STOP_AND_RESET_ENCODER
            else DcMotor.RunMode.RUN_USING_ENCODER
        if (gamepad.a) {
            wristAlignment = WristAlignment.FLOOR
            RobotLog.i("wrist aligned with FLOOR")
        }
        if (gamepad.y) {
            wristAlignment = WristAlignment.EASEL
            RobotLog.i("wrist aligned with EASEL")
        }
        if (gamepad.b) {
            armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            wristOffset = 0.0
            RobotLog.i("arm position reset, wrist offset reset")
        } else armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        if (gamepad.back) {}

        if (!dpadWasUsed) {
            wristOffset += when {
                gamepad.dpad_up -> WRIST_LARGE_INCREMENT
                gamepad.dpad_down -> -WRIST_LARGE_INCREMENT
                gamepad.dpad_left -> -WRIST_SMALL_INCREMENT
                gamepad.dpad_right -> WRIST_SMALL_INCREMENT
                else -> 0.0
            }
        }

        dpadWasUsed = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right

        /*
        if (button && !buttonWasPressed) {
            buttonWasPressed = true
            launcher.toggle()
        }
        else if (!button) buttonWasPressed = false
         */

        val wristTargetPosition =
            if (wristAlignment != null) relativeWristPosition(armMotor.currentPosition, wristAlignment!!, wristOffset.toRadians())
            else wrist.position

        if(wristAlignment != null) wrist.position = relativeWristPosition(armMotor.currentPosition, wristAlignment!!)
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM))
        telemetry.addData("Arm Motor Target (PPR)", armMotor.targetPosition)
        telemetry.addData("Arm Motor Actual (PPR)", armMotor.currentPosition)
        telemetry.addData("Arm Angle (degrees)", encoderPositionToDegrees(armMotor.currentPosition, ARM_ENCODER_RES))
        telemetry.addData("Wrist Position", wrist.position)
        telemetry.addData("Wrist Alignment", wristAlignment ?: "null")
    }

    fun getPlacementInfo(pixelRow: Int): PlacementInfo {
        val beta = asin(((D1 + pixelRow * D2 - L3) * sin(PI / 3) + L2 * cos(PI / 3) - H) / R)
        val alpha = PI / 2 + PI / 3 - GAMMA - beta
        val distToBase = R * cos(beta) + L2 * sin(PI / 3) + cos(PI / 3) * (L3 - D1 - pixelRow * D2)
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

    fun relativeWristPosition(armPosition: Int, target: WristAlignment, manualOffset: Double = 0.0): Double {
        wristAlignment ?: return wrist.position
        val theta = when(target) {
            WristAlignment.EASEL -> PI / 3
            WristAlignment.FLOOR -> 0.0
        }
        val armAngle = encoderPositionToDegrees(armPosition, ARM_ENCODER_RES) // in degrees
        val servoAngle = 11*PI/16 + theta - GAMMA - armAngle.toRadians() + manualOffset // radians
        return degreesToServoPosition(servoAngle.toDegrees(), ServoRanges.WRIST) // servo position value
    }

    fun autoCalibrate(abortCondition: () -> Boolean = { false }) {
        opMode.log("Attempting to automatically calibrate the arm motor")
        wrist.position = relativeWristPosition(0, WristAlignment.FLOOR)
        while(distanceSensor.getDistance(DistanceUnit.CM) > ARM_HEIGHT && armMotor.isBusy || abortCondition()) { // TODO: FIND THE REAL ARM_HEIGHT_VALUE
            armMotor.power = -0.2
            telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM))
            telemetry.addData("Arm Motor", armMotor.currentPosition)
            telemetry.addData("Wrist Servo", wrist.position)
            telemetry.update()
        }
        armMotor.power = 0.0
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Thread.sleep(10)
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        opMode.log("Arm motor successfully calibrated")
    }

    fun drop(target: WristAlignment) {
        opMode.log("Attempting to drop the arm")
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        armMotor.power = -0.2
        while(distanceSensor.getDistance(DistanceUnit.CM) > 6 && opMode.opModeIsActive()) {
            wrist.position = relativeWristPosition(armMotor.currentPosition, target)
            opMode.idle()
        }
        armMotor.power = 0.0
        telemetry.update()
        opMode.log("Arm motor successfully dropped?")
    }

    fun raise() = armMotor.moveTo(200)


    fun stow() {
        wristAlignment = null
        wrist.position = ArmConfig.WRIST_STOW_POSITION
    }

    fun DcMotor.moveTo(encoderPosition: Int) {
        targetPosition = encoderPosition
        mode = DcMotor.RunMode.RUN_TO_POSITION
        power = 0.2
    }

    companion object {
        const val ARM_ENCODER_RES = 2786.2 * 2 // PPR of motor * 2:1 gearing ratio
        const val ARM_HEIGHT = 24.0 // in CM
        // math
        val GAMMA = atan2(16.0, 283.0)
        const val L2 = 67.88
        const val L3 = 59.56
        const val R = 336.0
        const val H = 287.75
        const val D1 = 220.0
        const val D2 = 88.9
        // control
        const val WRIST_LARGE_INCREMENT = 15.0 // degrees
        const val WRIST_SMALL_INCREMENT = 7.5 // degrees
    }

    @Config
    object ArmConfig {
        @JvmField var WRIST_STOW_POSITION = 0.0 // TODO: FIND VALUE
    }
}
