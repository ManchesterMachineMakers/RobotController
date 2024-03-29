package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import com.rutins.aleks.diagonal.Subject
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.autonomous.path.motorEncoderEventsPerRevolution
import org.firstinspires.ftc.teamcode.util.OvercurrentProtection
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.clamp
import org.firstinspires.ftc.teamcode.util.degreesToServoPosition
import org.firstinspires.ftc.teamcode.util.encoderPositionToDegrees
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.powerCurve
import org.firstinspires.ftc.teamcode.util.toDegrees
import org.firstinspires.ftc.teamcode.util.toRadians
import kotlin.math.*

// Arm subassembly control
class Arm(opMode: OpMode) : Subject, Subassembly(opMode, "Arm") {
    val armMotor = hardwareMap.dcMotor.get("arm") as DcMotorEx
    val distanceSensor = hardwareMap.get(DistanceSensor::class.java, "intake_distance")
    val wrist = hardwareMap.servo.get("wrist")
    var wristAlignment: WristAlignment? = WristAlignment.EASEL
    var wristOffset = 0.0 // in degrees
    var dpadWasUsed = false

    val slowCoefficient: Double
        get() {
            val distance = distanceSensor.getDistance(DistanceUnit.CM)
            return if(distance < ARM_SLOW_DISTANCE_THRESHOLD) {
                opMode.log(
                    "Distance from object is less that %.1f at %.1f"
                        .format( ARM_SLOW_DISTANCE_THRESHOLD, distance)
                )
                distance / ARM_SLOW_DISTANCE_THRESHOLD
            } else {
                1.0
            }
        }

    init {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER // Resets the encoder (distance tracking)
        Thread.sleep(10)
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Wrist servo configuration
        wrist.scaleRange(WRIST_SCALE_RANGE.first, WRIST_SCALE_RANGE.second) // 53% of 300-degree range
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
        val leftY = gamepad.left_stick_y.toDouble()

        if (!armMotor.isOverCurrent) // lock out controls if overcurrent
            armMotor.power = powerCurve(
                    if(leftY < 0) { // down
                        leftY * clamp(getSlowCoefficient(20.0), 0.2, 1.0)
                    } else { // up/rest
                        leftY
                    }
            )

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
        if (gamepad.back) stow { !gamepad.atRest() }

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

        val wristTargetPosition =
            if (wristAlignment != null) relativeWristPosition(armMotor.currentPosition, wristAlignment!!, wristOffset.toRadians())
            else wrist.position

        wrist.position = wristTargetPosition

        telemetry.addData("Arm angle (degrees)", encoderPositionToDegrees(armMotor.currentPosition, ARM_ENCODER_RES))
        telemetry.addData("Arm encoder position", armMotor.currentPosition)
        telemetry.addData("Wrist alignment", wristAlignment ?: "null")
        telemetry.addData("Wrist position", "actual %.2f, target %.2f", wrist.position, wristTargetPosition)
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

    fun relativeWristPosition(armPosition: Int, target: WristAlignment, manualOffset: Double = 0.0): Double {
        wristAlignment ?: return wrist.position
        val theta = when(target) {
            WristAlignment.EASEL -> PI / 3
            WristAlignment.FLOOR -> 0.0
        }
        val armAngle = encoderPositionToDegrees(armPosition, ARM_ENCODER_RES) // in degrees
        val servoAngle = 11*PI/16 + theta - GAMMA - armAngle.toRadians() + manualOffset // radians
        return degreesToServoPosition(servoAngle.toDegrees(), WRIST_SCALE_RANGE) // servo position value
    }

    /**
     * @return the coefficient to slow down the arm, based on the distance sensor's value
     * @param distanceThreshold the distance when the arm will start slowing down
     * @param distanceUnit the unit distanceThreshold uses
     */
    fun getSlowCoefficient(distanceThreshold: Double, distanceUnit: DistanceUnit = DistanceUnit.CM): Double {
        wristAlignment ?: return 1.0
        val distance = distanceSensor.getDistance(distanceUnit)
        if(distance == 6553.5) {
            opMode.log("WARNING: Issue with distance sensor, arm-auto-slowing is not operational")
            return 1.0
        }
        return if(distance < distanceThreshold) {
            opMode.log(
                "Distance from object is less that %.1f at %.1f"
                    .format(distanceThreshold, distance)
            )
            distance / distanceThreshold
        } else {
            1.0
        }
    }

    fun drop() { // TODO: ALEKS PLEASE MAKE THIS WORK WITH DISTANCE SENSOR
//        while(!touchSensor.isPressed && armMotor.isBusy) {
//            armMotor.power = -0.2
//            telemetry.addData("Touch Sensor", touchSensor.isPressed)
//            telemetry.addData("Arm Motor", armMotor.currentPosition)
//            telemetry.addData("Wrist Servo", wrist.position)
//            telemetry.update()
//        }
//        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//        armMotor.power = 0.0
//        // go again, zeroed so the wrist position calculates correctly
//        telemetry.addLine("Zeroed the Arm Motor")
//        telemetry.log()
//        Thread.sleep(10)
//
//        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
//
//        while(!touchSensor.isPressed) {
//            armMotor.power = -0.2
//            wrist.position = relativeWristPosition(armMotor.currentPosition, WristAlignment.FLOOR)
//            Thread.sleep(10)
//        }
//        armMotor.power = 0.0
//        telemetry.update()
    }

    fun raise() = armMotor.moveTo(200)

    override fun telemetry() {
        super.telemetry()
        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM))
        telemetry.addData("Arm Motor","target %.2f, actual %.2f", armMotor.targetPosition, armMotor.currentPosition)
        telemetry.addData("Wrist Servo", wrist.position)
    }

    /**
     * @param ifAbort a check that returns true when an abort is necessary
     */
    fun stow(ifAbort: () -> Boolean) {
        opMode.log("Attempting to auto-stow robot arm")
        wristAlignment = null
        wrist.position = WRIST_STOW_POSITION
        val timer = ElapsedTime()
        while(!ifAbort()) {
            val current = armMotor.getCurrent(CurrentUnit.AMPS)
            if(current < ARM_AUTOSTOW_CURRENT) { // TODO: test on main bot to verify value.
                armMotor.power = 0.2
                opMode.log(
                    "Arm current (AMPS) is below $ARM_AUTOSTOW_CURRENT (at %.2f): continuing auto-stow at time: %.2f"
                    .format(current, timer.seconds())
                )
            } else {
                armMotor.power = 0.0
                opMode.log(
                    "Arm current (AMPS) exceeded $ARM_AUTOSTOW_CURRENT (at %.2f): finishing auto-stow at time: %.2f"
                    .format(current, timer.seconds())
                )
                break
            }
            if(timer.seconds() >= 5.0) {
                armMotor.power = 0.0
                opMode.log("Aborting arm auto-stow; time exceeded 5.0 seconds")
                break
            }
            Thread.sleep(100)
        }
        if(ifAbort()) opMode.log("Aborting arm auto-stow: received command")
        armMotor.power = 0.0
    }

    fun DcMotor.moveTo(encoderPosition: Int) {
        targetPosition = encoderPosition
        mode = DcMotor.RunMode.RUN_TO_POSITION
        power = 0.2
    }

    companion object {
        const val WRIST_LARGE_INCREMENT = 15.0 // degrees
        const val WRIST_SMALL_INCREMENT = 7.5 // degrees
        // config values
        val WRIST_SCALE_RANGE = Pair(0.25, 0.78)
        const val WRIST_STOW_POSITION = 0.0 // TODO: FIND VALUE
        const val ARM_AUTOSTOW_CURRENT = 2.0
        const val ARM_SLOW_DISTANCE_THRESHOLD = 30 // CM at which the arm will begin to slow down
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
