package org.firstinspires.ftc.teamcode.util.pathfinder.collision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.pathfinder.DistanceSensorManager
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.*
import kotlin.concurrent.fixedRateTimer

class CollisionDetector(private val opMode: LinearOpMode) : Subassembly {
    private val distanceSensorManager = KtHardware.get<DistanceSensorManager>(opMode)
    private var handlers: MutableMap<String, CollisionHandler> = mutableMapOf()
    private var timer: Timer = lookOut()
    private var stopRequested = false
    var collidingFront = false
        private set
    var collidingLeft = false
        private set
    var collidingRight = false
        private set

    fun lookOut(): Timer {
        // create a fixed rate timer that checks the distance sensor every 100ms
        // after a 100ms delay
        val fixedRateTimer = fixedRateTimer(name = "lookout-timer", initialDelay = 100, period = 100) {
            if (opMode.opModeIsActive() && !opMode.isStopRequested) {
                var distanceValues: DoubleArray = DoubleArray(5)
                val sensor: DistanceSensor? = distanceSensorManager?.frontIR
                val tolerance = 10.0

                var i: Int = 0
                while (i < 5 && (opMode.opModeIsActive() && !opMode.isStopRequested)) {
                    val distance: Double = sensor?.getDistance(DistanceUnit.CM)
                            ?: 100.toDouble()
                    distanceValues.set(i, distance)
                    i++
                }
                // average those values
                // should probably pass this as a lambda to a reduce() expression.
                distanceValues.sort()
                distanceValues.dropLast(1)
                distanceValues.reverse()
                distanceValues.dropLast(1)
                val distance = distanceValues.average();

                RobotLog.v("Polling in a Timer Task")
                collidingFront = poll(distance, tolerance, Collision.Front, Collision.EndedFront, collidingFront)
            }
        }
        return fixedRateTimer
    }

    fun stop() {
        timer.cancel()
    }

    fun stopThread() { stopRequested = true }

    private fun poll(distance: Double, tolerance: Double, collision: Collision, endCollision: Collision, colliding: Boolean) : Boolean {
        RobotLog.i("Distance measured: " + distance.toString())
        if (distance < tolerance && !colliding) {
            RobotLog.v("Collision imminent!")
            dispatch(collision)
            return true
        }
        if (distance > tolerance && colliding) {
            RobotLog.v("End collision, phew!")
            dispatch(endCollision)
            return false
        }
        RobotLog.v("Still colliding? " + colliding.toString())
        return colliding
    }

//        if(distanceSensorManager?.frontIR?.getDistance(DistanceUnit.CM)?:100.toDouble() < 10 && !collidingFront) {
//            dispatch(Collision.Front)
//            collidingFront = true
//        }
//        if(distanceSensorManager?.leftIR?.getDistance(DistanceUnit.CM)?:100.toDouble() < 10 && !collidingLeft) {
//            dispatch(Collision.Left)
//            collidingLeft = true
//        }
//        if(distanceSensorManager?.rightIR?.getDistance(DistanceUnit.CM)?:100.toDouble() < 10 && !collidingRight) {
//            dispatch(Collision.Right)
//            collidingRight = true
//        }
//        if(distanceSensorManager?.frontIR?.getDistance(DistanceUnit.CM)?:100.toDouble() > 10 && collidingFront) {
//            dispatch(Collision.EndedFront)
//            collidingFront = false
//        }
//        if(distanceSensorManager?.leftIR?.getDistance(DistanceUnit.CM)?:100.toDouble() > 10 && collidingLeft) {
//            dispatch(Collision.EndedLeft)
//            collidingLeft = true
//        }
//        if(distanceSensorManager?.rightIR?.getDistance(DistanceUnit.CM)?:100.toDouble() > 10 && collidingRight) {
//            dispatch(Collision.EndedRight)
//            collidingRight = true
//        }
//    }

    private fun dispatch(collision: Collision) = runBlocking {
        RobotLog.i("Dispatching in a blocking thread")
        for (handler in handlers) {
            launch {
                // RobotLog.v("Dispatching a non-blocking handler")
                handler.value(collision)
            }
        }
        RobotLog.i("Exiting the Dispatch blocking thread")
    }

    fun observe(handler: Pair<String, CollisionHandler>) {
        handlers[handler.first] = handler.second
    }

    fun removeObserver(name: String) {
        handlers.remove(name)
    }
}

enum class Collision { Front, Left, Right, EndedFront, EndedLeft, EndedRight }

typealias CollisionHandler = (Collision) -> Unit