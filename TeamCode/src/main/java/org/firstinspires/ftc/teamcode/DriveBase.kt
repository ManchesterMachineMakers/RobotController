package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import org.firstinspires.ftc.teamcode.util.Conversions
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.sin

object DriveBase : CustomBlocksOpModeCompanion() {

    private var config: Configuration? = null

    @JvmStatic
    fun use(config: Configuration) {
        DriveBase.config = config
    }

    /**
     * Constants to represent drive speeds that are useful.  Set power values for each in initDriveSpeedConfiguration().
     */
    enum class DriveSpeed {
        STOP, ANGLE_CORRECTION, SLOW, FAST, MAX
    }

    /**
     * A predefined set of travel directions. Set motor configurations for each direction in Configuration.init().
     */
    enum class TravelDirection {
        base, forward, reverse, pivotLeft, pivotRight, strafeLeft, strafeLeftForward, strafeLeftBackward, strafeRight, strafeRightForward, strafeRightBackward, pitch
    }

    class Configuration(motors: Array<String>, val config: HashMap<TravelDirection, Array<DcMotorSimple.Direction?>>) {
        val motors = motors.map { name -> RobotConfig.getHardware<DcMotor>(name) }
    }

    // Wheel diameter tells us how far it will travel.
    // Metric
    var wheelDiameterMM = 96.0
    var wheelRotationDistanceMM = Math.PI * wheelDiameterMM
    var motorRotationsPerMM = 1 / wheelRotationDistanceMM

    // English
    var wheelDiameterInches: Double = Conversions.mmToInches(wheelDiameterMM)
    var wheelRotationDistanceInches = Math.PI * wheelDiameterInches
    var motorRotationsPerInch = 1 / wheelRotationDistanceInches

    var wheelBaseWidth = 16.0 // inches

    var wheelBaseLengthMM = 360.0
    var inchwormFrontMM = 216.0 // front to pivot point

    var inchwormRearMM = 144.0 // rear to pivot point


    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-223-rpm-3-3-5v-encoder/
    var motorEncoderEventsPerRotation = 753.2
    var motorEncoderEventsPerInch = motorRotationsPerInch * motorEncoderEventsPerRotation
    var motorEncoderEventsPerMM = motorRotationsPerMM * motorEncoderEventsPerRotation

    var maxPower = 1.0 // range of 0 to 1
    /**
     * Returns the defined array of directions for motors, as set in the initMotorConfigurations method.
     * @param travelDirection which direction the bot should travel
     * @return the defined motor directions, or null if no matching configuration has been defined.
     */
    fun getMotorConfigurations(travelDirection: TravelDirection?): Array<DcMotorSimple.Direction?>? {
        return config?.config?.getOrDefault(travelDirection, null)
    }

    val driveSpeedConfigurations = hashMapOf(
        DriveSpeed.STOP to 0.0,
        DriveSpeed.SLOW to 0.2,
        DriveSpeed.FAST to 0.8,
        DriveSpeed.MAX to 1.0
    )

    /**
     * Returns the power value for the given drive speed, or null.
     * @param driveSpeed how fast we should go - use the enum
     * @return the current power for the drive speed
     */
    @ExportToBlocks(
        comment = "Returns the power value for the given drive speed, or null.",
        tooltip = "Returns the power value for the given drive speed, or null.",
        parameterLabels = ["Drive Speed"]
    )
    @JvmStatic fun getDriveSpeedPower(driveSpeed: DriveSpeed?): Double {
        return driveSpeedConfigurations.getOrDefault(driveSpeed, 0.0)
    }

    /**
     * Configure motors in initMotorConfigurations.
     * @return the series, in order as configured, of motor objects that are relevant to this drivebase.
     */
    fun getMotors(): List<DcMotor?>? {
        return config?.motors
    }

    /**
     * Initialize all motors to run using encoders.
     * Set travel to forward and power to  0.
     */
    @ExportToBlocks(
        comment = "Initialize all motors to run using encoders. Set travel to forward and power to 0."
    )
    @JvmStatic
    fun initHardware() {

        // Output all configuration information for the drive base.
        RobotLog.i("*******  Drive Base Configuration *******")
        RobotLog.i("Wheel diameter (inches): $wheelDiameterInches")
        RobotLog.i("Floor distance for one wheel rotation (inches): $wheelRotationDistanceInches")
        RobotLog.i("Rotations per inch of floor distance: $motorRotationsPerInch")
        RobotLog.i("Wheel base width (inches): $wheelBaseWidth")
        RobotLog.i("Motor encoder events per wheel rotation: $motorEncoderEventsPerRotation")
        RobotLog.i("Motor encoder events per inch of floor distance: $motorEncoderEventsPerInch")
        RobotLog.i("Maximum power to motors: $maxPower")
        RobotLog.i(
            "Base wheel directions: " +
                getMotorConfigurations(
                    TravelDirection.base
                ).toString()
        )
        RobotLog.i("*******  *******")
        setRunMode(RunMode.STOP_AND_RESET_ENCODER)
        setStopMode(ZeroPowerBehavior.BRAKE)
        setTravelDirection(TravelDirection.base)

        // set motor power to 0.
        stop()
    }

    /**
     * Remove instances of motors, travel directions, speeds
     */
    @ExportToBlocks
    @JvmStatic
    fun cleanup() {
        stop()
    }

    /**
     * Set the runmode on all motors.
     * @param runMode how the motors should run - use the enum.
     * @return boolean success
     */
    @ExportToBlocks @JvmStatic
    fun setRunMode(runMode: RunMode?): Boolean {
        return try {
            val motors = getMotors()!!
            for (motor in motors) {
                motor?.mode = runMode
            }
            true
        } catch (e: Exception) {
            RobotLog.i(e.message)
            false
        }
    }

    /**
     * Set the zero power behavior on all motors.
     * @param behavior zero power behavior - use the enum
     * @return boolean success
     */
    @ExportToBlocks @JvmStatic
    fun setStopMode(behavior: ZeroPowerBehavior?): Boolean {
        return try {
            for (motor in getMotors()!!) {
                motor?.zeroPowerBehavior = behavior
            }
            true
        } catch (e: Exception) {
            RobotLog.i(e.message)
            false
        }
    }

    /**
     * Returns the encoder values for all motors.
     * @return current positions of all motors, in order configured.
     */
    @ExportToBlocks @JvmStatic
    fun getEncoderPositions(): Array<Int?> {
        val motors = getMotors()!!
        val positions = Array<Int?>(motors.size) { 0 }
        for (i in motors.indices) {
            positions[i] = motors[i]?.currentPosition
        }
        return positions
    }

    /**
     * Set the target position tolerance on each motor.
     * @param tolerance position tolerance
     * @return position tolerance from each motor, in order configured.
     */
    @ExportToBlocks @JvmStatic
    fun setPositionTolerance(tolerance: Int): IntArray {
        // set position tolerance
        val motors = getMotors()!!
        val tolerances = IntArray(motors.size)
        for (i in motors.indices) {
            (motors[i] as DcMotorEx).targetPositionTolerance = tolerance
            tolerances[i] = (motors[i] as DcMotorEx).targetPositionTolerance
        }
        return tolerances
    }

    /**
     * Returns the target position tolerance of each motor as an integer array.
     * This probably is unnecessary.  We can remove it if so.
     * @return position tolerance from each motor, in order configured.
     */
    @ExportToBlocks @JvmStatic
    fun getPositionTolerance(): IntArray {
        // set position tolerance
        val motors = getMotors()!!
        val tolerances = IntArray(motors.size)
        for (i in motors.indices) {
            tolerances[i] = (motors[i] as DcMotorEx).targetPositionTolerance
        }
        return tolerances
    }

    /**
     * Returns the current position of each motor as an integer array.
     */
    @ExportToBlocks @JvmStatic
    fun checkMotorPositions(): Array<Int?> {
        val motors = getMotors()!!
        val positions = Array<Int?>(motors.size) {0}
        for (i in motors.indices) {
            positions[i] = motors[i]?.currentPosition
        }
        return positions
    }

    /**
     * Stop each motor if it has reached its target.
     * @param targetTicks
     * @param tolerance
     * @return
     */
    @ExportToBlocks(parameterLabels = ["Target Ticks", "Tolerance"]) @JvmStatic
    fun stopOnTicks(targetTicks: IntArray, tolerance: Int): Array<Boolean> {
        val positions = checkMotorPositions()
        val motors = getMotors()!!
        // track which motors are stopped by this
        val isStopped = Array(motors.size) { false }
        for (i in motors.indices) {
            if (positions[i]!! >= targetTicks[i] + tolerance) {
                motors[i]?.power = 0.0
                isStopped[i] = true
            }
        }
        return isStopped
    }

    /**
     * Set the direction of travel to one of the predefined configurations.
     * @param travelDirection which way should the bot go?
     * @return  success
     */
    @ExportToBlocks @JvmStatic
    fun setTravelDirection(travelDirection: TravelDirection?): Boolean {
        // our motor configuration
        return try {
            val motors = getMotors()!!
            val directions = getMotorConfigurations(travelDirection)
            if (directions == null) {
                stop()
                return false
            }
            for (i in motors.indices) {
                motors[i]?.direction = directions[i]
            }
            true
        } catch (e: Exception) {
            stop()
            false
        }
    }

    @ExportToBlocks(parameterLabels = ["Directions"]) @JvmStatic
    fun setMotorDirections(directions: Array<DcMotorSimple.Direction?>): Boolean {
        try {
            val motors = getMotors()!!
            for (i in motors.indices) {
                motors[i]?.direction = directions[i]
            }
            return true
        } catch (ex: Exception) {
            RobotLog.e(ex.message)
        }
        return false
    }

    @ExportToBlocks @JvmStatic
    fun getMotorDirections(): Array<DcMotorSimple.Direction?>? {
        try {
            val motors = getMotors()!!
            val directions = arrayOfNulls<DcMotorSimple.Direction>(motors.size)
            for (i in motors.indices) {
                directions[i] = motors[i]?.direction
            }
            return directions
        } catch (ex: Exception) {
            RobotLog.e(ex.message)
        }
        return null
    }

    @ExportToBlocks @JvmStatic
    fun getTravelDirection(): TravelDirection? {
        try {
            val motors = getMotors()!!
            val directions = arrayOfNulls<DcMotorSimple.Direction>(motors.size)
            for (i in motors.indices) {
                directions[i] = motors[i]?.direction
            }
            for (k in config?.config?.keys!!) {
                val config = getMotorConfigurations(k)
                var match = true
                for (i in directions.indices) {
                    match = match && config?.get(i) == directions[i]
                }
                if (match) {
                    return k
                }
            }
        } catch (ex: Exception) {
            RobotLog.e(ex.message)
        }
        return null
    }

    @ExportToBlocks @JvmStatic
    fun isBusy(): Boolean {
        val motors = getMotors()!!
        for (motor in motors) {
            if (motor?.mode == RunMode.RUN_TO_POSITION) {
                if (motor.isBusy) {
                    return true
                }
            } else {
                if (motor?.mode != RunMode.STOP_AND_RESET_ENCODER && motor?.power != 0.0) {
                    //RobotLog.i(motor.getDeviceName() + " is busy - power is " + String.valueOf(motor.getPower()));
                    return true
                }
            }
        }
        //RobotLog.i("Motors are not busy.");
        return false
    }

    /**
     * Stop all motors - set power to 0.
     * @return power
     */
    @ExportToBlocks @JvmStatic
    fun stop(): Double {
        val motors = getMotors()!!
        for (motor in motors) {
            motor?.power = 0.0
        }
        return 0.0
    }

    /**
     * Is the drivebase really stopped, by using the runmode STOP_AND_RESET_ENCODER?
     * @return
     */
    @ExportToBlocks @JvmStatic
    fun isStopped(): Boolean {
        val motors = getMotors()!!
        var stopped = true
        for (motor in motors) {
            stopped = stopped && motor?.mode == RunMode.STOP_AND_RESET_ENCODER
        }
        return stopped
    }

    /**
     * Go in one of the set directions at the set speed.  Stops on exception.
     * @param direction which way should the bot go?
     * @param driveSpeed how fast?
     * @return the power to the motors
     */
    @ExportToBlocks @JvmStatic
    fun go(direction: TravelDirection?, driveSpeed: DriveSpeed?): Double {
        val power = getDriveSpeedPower(driveSpeed)
        return go(direction, power)
    }

    /**
     * Go in one of the set directions at the set power. Stops on exception.
     * @param direction  which way should the bot go?
     * @param power power value for the motors
     * @return the power to the motors
     */
    @ExportToBlocks(parameterLabels = ["Direction", "Power"]) @JvmStatic
    fun go(direction: TravelDirection?, power: Double): Double {
        val directions = getMotorConfigurations(direction)
        return go(directions, power)
    }

    /**
     * Send the motors in any given direction at the set power.  Stops on exception.
     * @param directions  which way should each motor turn?
     * @param power power value for the motors
     * @return the power to the motors
     */
    @ExportToBlocks(parameterLabels = ["Directions", "Power"])
    fun go(directions: Array<DcMotorSimple.Direction?>?, power: Double): Double {
        return try {
            if (directions == null) {
                // we asked for a direction we haven't defined.
                RobotLog.i("DriveBase::go: Cannot go in an undefined direction.")
                return stop()
            }
            RobotLog.i("Power: $power Directions: $directions")
            val motors = getMotors()!!
            for (i in motors.indices) {
                // motors should always be set to the base direction.
                // power varies by the directions given.  If forward, positive.
                // if reverse, negative.
                val motorPower = power * if (directions[i] == DcMotorSimple.Direction.FORWARD) 1 else -1
                motors[i]?.power = motorPower
            }
            power
        } catch (e: Exception) {
            RobotLog.i(e.message)
            stop()
        }
    }

    /**
     * Set power levels on each motor separately.  Allows finer navigational control of travel paths.
     * Useful especially for a game controller interface.
     * @param powerLevels power level for motor, in order as configured.
     * @return the power level of the first motor.
     */
    @ExportToBlocks(parameterLabels = ["Power Levels"]) @JvmStatic
    fun go(powerLevels: DoubleArray): Double {
        val motors = getMotors()!!

        // if there aren't enough power levels in the array, add 0 on the end.
        if (powerLevels.size < motors.size) {
            stop()
            throw ArrayIndexOutOfBoundsException("There are more motors than power levels specified.")
        }

        // set each motor in the array to its pair in the power array.
        for (i in motors.indices) {
            motors[i]?.power = powerLevels[i]
        }

        // return the first motor's power level.
        return if (powerLevels.isNotEmpty()) powerLevels[0] else 0.0
    }

    /**
     * Set all motors to a powerlevel.  We assume that a direction and runmode are already set.
     * @param powerLevel
     * @return
     */
    @ExportToBlocks(parameterLabels = ["Power Level"]) @JvmStatic
    fun go(powerLevel: Double): Double {
        val motors = getMotors()!!
        for (i in motors.indices) {
            motors[i]?.power = powerLevel
        }
        return powerLevel
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param travelDirection which way should the bot go?
     * @param power the power to each motor
     * @param encoderTicks the distance each motor  should go, by the encoders
     */
    @ExportToBlocks(parameterLabels = ["Travel Direction", "Power", "Encoder Ticks"]) @JvmStatic
    fun go(travelDirection: TravelDirection?, power: Double, encoderTicks: Int): Double {
        return go(travelDirection, power, encoderTicks, 0)
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param travelDirection which way should the bot go?
     * @param power the power to each motor
     * @param encoderTicks the distance each motor  should go, by the encoders
     * @param tolerance the tolerance to set on each motor
     */
    @ExportToBlocks(parameterLabels = ["Travel Direction", "Power", "Encoder Ticks", "Tolerance"]) @JvmStatic
    fun go(
        travelDirection: TravelDirection?,
        power: Double,
        encoderTicks: Int,
        tolerance: Int
    ): Double {
        val driveConfiguration = getMotorConfigurations(travelDirection)!!
        val wheelEncoderTicks = IntArray(driveConfiguration.size)
        val motors = getMotors()!!
        for (i in motors.indices) {
            wheelEncoderTicks[i] =
                encoderTicks * if (driveConfiguration[i] == DcMotorSimple.Direction.FORWARD) 1 else -1
        }
        return go(power, wheelEncoderTicks, tolerance)
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param power the power to each motor (we can make this a list if  needed)
     * @param encoderTicks a  list of  encoder tick values,  distances  that each motor should go by the encoders.
     * @param tolerance the tolerance to set on each motor
     */
    @ExportToBlocks(parameterLabels = ["Power", "Encoder Ticks", "Tolerance"]) @JvmStatic
    fun go(power: Double, encoderTicks: IntArray, tolerance: Int): Double {
        val currentPositions = getEncoderPositions()
        RobotLog.i("Current Positions when driving ticks: $currentPositions")
        RobotLog.i("Power: $power Encoder Ticks: $encoderTicks")
        RobotLog.i("Tolerance: $tolerance")
        val motors = getMotors()!!

        // set each motor, based on drive configuration/travel direction
        for (i in motors.indices) {
            motors[i]?.mode = RunMode.RUN_USING_ENCODER
            motors[i]?.targetPosition =
                encoderTicks[i] //calcTargetPosition(motors[i].getDirection(), currentPositions[i], encoderTicks[i]));
            (motors[i] as DcMotorEx).targetPositionTolerance = tolerance
            motors[i]?.mode = RunMode.RUN_TO_POSITION
            motors[i]?.power = power
        }
        return power
    }

    /**
     * Set target position for all motors at once.
     * @param encoderTicks represents the target position for each motor.
     */
    @ExportToBlocks @JvmStatic
    fun setTargetPositions(encoderTicks: Int): Double {
        val motors = getMotors()!!
        for (i in motors.indices) {
            motors[i]?.targetPosition = encoderTicks
        }
        return encoderTicks.toDouble()
    }

    /**
     * Get target position for all motors at once.
     */
    @ExportToBlocks @JvmStatic
    fun getTargetPositions(): IntArray {
        val motors = getMotors()!!
        val targetTicks = IntArray(motors.size)
        for (i in motors.indices) {
            targetTicks[i] = motors[i]?.targetPosition ?: 0
        }
        return targetTicks
    }

    /**
     * Utility method for use in setting encoder values.  Is this correct for reverse directions??
     * NOT NEEDED - this is handled in the DCMotorImpl.
     * @param motorDirection which way is the motor turning?
     * @param currentPosition where are we now?
     * @param encoderTicks how far should we go?
     * @return the encoder value we're shooting for.
     */
    private fun calcTargetPosition(
        motorDirection: DcMotorSimple.Direction,
        currentPosition: Int,
        encoderTicks: Int
    ): Int {
        return when (motorDirection) {
            DcMotorSimple.Direction.FORWARD -> currentPosition + encoderTicks
            DcMotorSimple.Direction.REVERSE -> currentPosition - encoderTicks
            else -> currentPosition
        }
    }

    /**
     * Returns ticks for the encoder to run when we want to pivot around our own middle.
     * @param theta is the angle in degrees that we should be turning.
     * @return the encoder ticks needed to move the bot that far.
     */
    fun getEncoderValueForRobotPivotAngle(theta: Float): Double {
        // calculate the arc of the pivot
        val radius = wheelBaseWidth / 2
        val arcLength = radius * theta * Math.PI / 180 // length in inches

        // now that we have the arcLength, we figure out how many wheel rotations are needed.
        return getEncoderValueForRobotInches(arcLength)
    }

    /**
     * Returns ticks for the encoder to run when we want to cover a distance.
     * @param inchesToTravel how far in inches should the bot go?
     * @return  the encoder ticks needed to move the bot that far.
     */
    fun getEncoderValueForRobotInches(inchesToTravel: Double): Double {
        // number of ticks for the encoder
        val ticks = inchesToTravel * motorEncoderEventsPerInch
        RobotLog.i("Calculated ticks for $inchesToTravel as $ticks")
        return ticks
    }


    /**
     * Returns ticks for the encoder to run when we want to cover a distance.
     * @param mmToTravel how far in inches should the bot go?
     * @return  the encoder ticks needed to move the bot that far.
     */
    fun getEncoderValueForRobotMillimeters(mmToTravel: Double): Double {
        val ticks = mmToTravel * motorEncoderEventsPerMM
        RobotLog.i("Calculated ticks for $mmToTravel as $ticks")
        return ticks
    }

    /**
     * Return the number of ticks difference along the floor between
     * the length between the wheels  when flat and when pitched at angle theta.
     */
    fun getEncoderValueForPitchAngle(theta: Double): Double {
        // fr = segment length: front to rear when base is flat
        // rp  = segment length: rear to pivot point
        // fp = segment length: front to pivot point
        // h = segment length: height of pivot point from flat
        // rbp = segment length: rear to pivot along the floor
        // fbp = segment length: front to pivot along the floor
        val rp = inchwormRearMM
        val fp = inchwormFrontMM
        val fr = rp + fp
        val rbp = rp * cos(theta)
        val h = rp * sin(theta)
        val ftheta = asin(h / fp)
        val fbp = fp * cos(ftheta)
        return (fr - (rbp + fbp)) * motorEncoderEventsPerMM
    }
    //TODO: Get Isaac's encoder value for MM routine.
}