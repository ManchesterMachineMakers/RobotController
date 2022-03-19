package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.drivebase.DriveBase
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.pid.runPID;
import org.firstinspires.ftc.teamcode.util.pathfinder.collision.Collision
import org.firstinspires.ftc.teamcode.util.pathfinder.collision.CollisionDetector
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

data class PathfinderResult(val finalLocation: MatrixF, val finalRotation: Double)

class Pathfinder(private val opMode: LinearOpMode) : Subassembly {
    val localization = KtHardware.get<Localization>(opMode)
    val driveBase = KtHardware.get<DriveBase>(opMode)
    val collision = KtHardware.get<CollisionDetector>(opMode)
    val pivotTolerance: Double = 1.0

    class NoPositionError : Error("Could not get robot position")

    init {
        // TODO: move into drive base
        driveBase?.wheelBaseWidth = 12.5
    }

    fun getPivotAngleForEncoderTicks(ticks: Double): Double {
        // TODO: move into drive base
        val inches = ticks / driveBase?.motorEncoderEventsPerInch!!
        val radius = driveBase.wheelBaseWidth / 2
        return (180 * inches) / (radius * Math.PI)
    }

    fun pivotTo(targetAngle: Double, currentAngle: Double): Double {
        val direction = if(targetAngle < currentAngle) DriveBase.TravelDirection.pivotRight else DriveBase.TravelDirection.pivotLeft
        val angleSign = if(targetAngle < currentAngle) 1                                    else -1
        driveBase?.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
        val ticks = driveBase?.getEncoderValueForRobotPivotAngle(abs(targetAngle - currentAngle).toFloat())
        val result = runPID(opMode, 0.0, ticks ?: 0.0, pivotTolerance, 100.0) { _, _, _, _ ->
            val power = calculateCorrection()
            driveBase?.go(direction, power)
            driveBase?.encoderPositions?.average() ?: ticks ?: 0.0
        }

        driveBase?.stop()

        return currentAngle + (getPivotAngleForEncoderTicks(result) * angleSign)
    }

    fun runTo(targetX: Float, targetY: Float, currentRotation: Double, currentLocationFallback: MatrixF? = null, clearance: Float = 0F, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW): PathfinderResult {
        val location = localization?.getRobotLocation() ?: currentLocationFallback ?: throw NoPositionError()
        val path = Path(location.toVector()[0], location.toVector()[1], targetX, targetY, currentRotation.toFloat(), clearance)
        var startingMotorPositions = driveBase?.checkMotorPositions()
        var currentMotorPositions = startingMotorPositions
        driveBase?.setStopMode(DcMotor.ZeroPowerBehavior.BRAKE)
        // Collision handlers
        collision?.observe("pathfinder_runTo" to {
            when(it) {
                Collision.Front -> driveBase?.stop()
                Collision.EndedFront -> driveBase?.go(driveBase.getDriveSpeedPower(speed))
                else -> {}
            }
        })

        // collision?.startThread()
        // pivot
        val pivotResult = pivotTo(path.heading, currentRotation)
        currentMotorPositions = driveBase?.checkMotorPositions()
        val newTranslation = calcNewTranslation(getAverageDistanceTraveled(startingMotorPositions, currentMotorPositions), path.heading)
        
        // run
        driveBase?.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
        driveBase?.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        driveBase?.go(
                if(path.direction > 0)
                    DriveBase.TravelDirection.reverse
                else
                    DriveBase.TravelDirection.forward,
                driveBase.getDriveSpeedPower(speed),
                Values.getTicks(path.distance)
        )
        while (driveBase?.isBusy == true) opMode.idle()
        collision?.removeObserver("pathfinder_runTo")
        // collision?.stopThread()
        collision?.stop()

        return PathfinderResult(localization?.getRobotLocation() ?: newTranslation.added(location), pivotResult)
    }

    fun runTo(target: Destination, currentRotation: Double, currentLocationFallback: MatrixF? = null, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) = 
        runTo(target.destX, target.destY, currentRotation, currentLocationFallback, target.clearanceRadius, speed)
    
    fun runTo(target: Destination, currentTransform: PathfinderResult, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) =
        runTo(target.destX, target.destY, currentTransform.finalRotation, currentTransform.finalLocation, target.clearanceRadius, speed)

    fun isWithinTolerance(a: Double, b: Double, t: Double) : Boolean {
        return abs(b - a) < abs(t)
    }

    fun getAverageDistanceTraveled(from: IntArray?, to: IntArray?) : Double {
        var acc : IntArray? = from
        // iterate through the motor positions in the from location and compare to the to location.
        return (driveBase?.encoderPositions?.average() ?: 0.0)
    }

    fun calcNewTranslation(distanceTraveled: Double, heading: Double) : OpenGLMatrix {
        // need closest 90 angle for heading calc
        // add/subtract from current location

        // QUADRANTS
        //      .   |   .
        //       \  |  /
        //    Q0  \h| /  Q3
        //         \|/ h = heading angle
        //--------------------
        //        h/|\
        //   Q1   / |h\  Q2
        //       /  |  \
        //      .   |   .

        // determine quadrant
        val quadrant: Int = (heading/90).toInt() // INT: 0 = less than 90, 1 = 91-180, 2 = 181-270, 3 = 271-360
        val quadrantHeading = heading % 90

        // SOH CAH TOA - y = opposite, x = adjacent
        val adjacent = cos(quadrantHeading) * distanceTraveled
        val opposite = sin(quadrantHeading) * distanceTraveled
        var x: Double = 0.0
        var y: Double = 0.0

        when (quadrant) {
            0 -> {
                x = -opposite
                y = adjacent
            }
            1 -> {
                x = -adjacent
                y = -opposite
            }
            2 -> {
                x = opposite
                y = -adjacent
            }
            3 -> {
                x = adjacent
                y = opposite
            }
            else -> {
                RobotLog.i("Getting current translation: heading not in a quadrant")
            }
        }

        // add or subtract based on quadrant from current location
        return OpenGLMatrix.translation(x.toFloat(), y.toFloat(), 0.0F)

    }
}