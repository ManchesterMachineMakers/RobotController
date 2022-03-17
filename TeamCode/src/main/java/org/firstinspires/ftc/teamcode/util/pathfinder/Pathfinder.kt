package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.teamcode.drivebase.DriveBase
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.pathfinder.collision.Collision
import org.firstinspires.ftc.teamcode.util.pathfinder.collision.CollisionDetector
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class Pathfinder(private val opMode: LinearOpMode) : Subassembly {
    val localization = KtHardware.get<Localization>(opMode)
    val driveBase = KtHardware.get<DriveBase>(opMode)
    val collision = KtHardware.get<CollisionDetector>(opMode)
    val pivotTolerance: Double = 1.0

    class NoPositionError : Error("Could not get robot position")

    fun pivotTo(targetAngle: Double, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) {
        val currentAngle = localization?.imu?.orientation?.psi ?: targetAngle
        val direction = if(targetAngle < currentAngle) DriveBase.TravelDirection.pivotRight else DriveBase.TravelDirection.pivotLeft
        driveBase?.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        driveBase?.go(direction, speed)
        while(!isWithinTolerance(localization?.imu?.orientation?.psi ?: targetAngle, targetAngle, pivotTolerance) && driveBase?.isBusy == true && opMode.opModeIsActive()) opMode.idle()
        driveBase?.stop()
    }

    fun runTo(targetX: Float, targetY: Float, currentLocationFallback: OpenGLMatrix? = null, clearance: Float = 0F, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) {
        val location = localization?.getRobotLocation() ?: currentLocationFallback ?: throw NoPositionError()
        val path = Path(location[0, 0], location[1, 0], targetX, targetY, localization?.imu?.orientation?.psi ?: 0.0, clearance)
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

        collision?.startThread()
        // pivot
        pivotTo(path.heading)
        currentMotorPositions = driveBase?.checkMotorPositions()
        calcNewTranslation(getAverageDistanceTraveled(startingMotorPositions, currentMotorPositions), localization?.imu?.orientation?.psi ?: path.heading)
        // run
        driveBase?.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
        driveBase?.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        driveBase?.go(
                if(path.direction < 0)
                    DriveBase.TravelDirection.reverse
                else
                    DriveBase.TravelDirection.forward,
                driveBase.getDriveSpeedPower(speed),
                Values.getTicks(path.distance)
        )
        while (driveBase?.isBusy == true) opMode.idle()
        collision?.removeObserver("pathfinder_runTo")
        collision?.stopThread()
    }

    fun runTo(target: Destination, currentLocationFallback: OpenGLMatrix? = null, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) {
        runTo(target.destX, target.destY, currentLocationFallback, target.clearanceRadius, speed)
    }

    fun isWithinTolerance(a: Double, b: Double, t: Double) : Boolean {
        return abs(a - b) > abs(t)
    }

    fun getAverageDistanceTraveled(from: IntArray?, to: IntArray?) : Int {
        var acc : IntArray? = from
        // iterate through the motor positions in the from location and compare to the to location.
        //TODO: take the average distance and return.
        return 0
    }

    fun calcNewTranslation(distanceTraveled: Int, heading: Double) : OpenGLMatrix {
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
        val quadrant : Int = (heading/90).toInt() // INT: 0 = less than 90, 1 = 91-180, 2 = 181-270, 3 = 271-360
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