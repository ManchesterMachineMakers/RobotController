package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.autonomous.path.motorEncoderEventsPerRevolution
import org.firstinspires.ftc.teamcode.util.OvercurrentProtection
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.clamp
import org.firstinspires.ftc.teamcode.util.degreesToServoPosition
import org.firstinspires.ftc.teamcode.util.encoderPositionToDegrees
import org.firstinspires.ftc.teamcode.util.powerCurve
import kotlin.math.*

// Arm subassembly control
class Arm(opMode: OpMode) : Subject, Subassembly(opMode, "Arm") {
    val armMotor = hardwareMap.dcMotor.get("arm") as DcMotorEx
    val wrist = hardwareMap.servo.get("wrist")
    val touchSensor = hardwareMap.touchSensor.get("intake")
    var wristAlignment: WristAlignment? = null

    val overcurrentProtection = OvercurrentProtection(armMotor, 5.0, {
        armMotor.targetPosition = 0
        armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }, { armMotor.setMotorDisable() })

    @Deprecated("This is now deprecated, use the Subassembly `PixelReleases.kt`")
    val rightRelease = hardwareMap.servo.get("right_release")
    @Deprecated("This is now deprecated, use the Subassembly `PixelReleases.kt`")
    val leftRelease = hardwareMap.servo.get("left_release")

    init {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER // Resets the encoder (distance tracking)
        Thread.sleep(10)
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Wrist servo configuration
        wrist.scaleRange(WRIST_SCALE_RANGE.first, WRIST_SCALE_RANGE.second) // 53% of 300-degree range
        wrist.direction = Servo.Direction.FORWARD
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

    fun control(gamepad: Gamepad) {
        val armMotor = armMotor

        if (!armMotor.isBusy) // lock out controls if busy
            armMotor.power = powerCurve(gamepad.left_stick_y.toDouble())

        armMotor.mode = // arm calibration
            if (gamepad.b) DcMotor.RunMode.STOP_AND_RESET_ENCODER
            else DcMotor.RunMode.RUN_USING_ENCODER

        if (gamepad.a) wristAlignment = WristAlignment.FLOOR
        if (gamepad.y) wristAlignment = WristAlignment.EASEL
        if (gamepad.back) stow()

        if (wristAlignment != null) wrist.position = relativeWristPosition(armMotor.currentPosition, wristAlignment!!)
    }

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

    fun relativeWristPosition(armPosition: Int, target: WristAlignment): Double {
        wristAlignment ?: return wrist.position
        val theta = when(target) {
            WristAlignment.EASEL -> 60
            WristAlignment.FLOOR -> 120
        }
        val armAngle = encoderPositionToDegrees(armPosition, ARM_ENCODER_RES) // in degrees
        val servoAngle = theta - GAMMA - armAngle // degrees
        return degreesToServoPosition(servoAngle, WRIST_SCALE_RANGE) // servo position value
    }

    fun drop() {
        while(!touchSensor.isPressed && armMotor.isBusy) {
            armMotor.power = -0.2
            telemetry.addData("Touch Sensor", touchSensor.isPressed)
            telemetry.addData("Arm Motor", armMotor.currentPosition)
            telemetry.addData("Wrist Servo", wrist.position)
            telemetry.update()
        }
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armMotor.power = 0.0
        // go again, zeroed so the wrist position calculates correctly
        telemetry.addLine("Zeroed the Arm Motor")
        telemetry.log()
        Thread.sleep(10)

        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        while(!touchSensor.isPressed) {
            armMotor.power = -0.2
            wrist.position = relativeWristPosition(armMotor.currentPosition, WristAlignment.FLOOR)
            Thread.sleep(10)
        }
        armMotor.power = 0.0
        telemetry.update()
    }

    fun raise() = armMotor.moveTo(200)

    fun stow() {
        val timer = ElapsedTime()
        wristAlignment = null
        wrist.position = WRIST_STOW_POSITION
        while(armMotor.getCurrent(CurrentUnit.AMPS) < 2.0 || timer.seconds() < 5 /*TODO: adjust if necessary*/) armMotor.power = 0.2
        armMotor.power = 0.0
    }

    override fun telemetry() {
        super.telemetry()
        telemetry.addData("Touch Sensor", touchSensor.isPressed)
        telemetry.addData("Arm Motor","target %.2f, actual %.2f", armMotor.targetPosition, armMotor.currentPosition)
        telemetry.addData("Wrist Servo", wrist.position)
    }

    fun DcMotor.moveTo(encoderPosition: Int) {
        targetPosition = encoderPosition
        mode = DcMotor.RunMode.RUN_TO_POSITION
        power = 0.2
    }

    companion object {
        // config values
        val WRIST_SCALE_RANGE = Pair(0.25, 0.78)
        const val WRIST_STOW_POSITION = 0.5 // TODO: find value
        // constants
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
