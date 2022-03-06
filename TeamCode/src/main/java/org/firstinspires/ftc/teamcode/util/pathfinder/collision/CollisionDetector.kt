package org.firstinspires.ftc.teamcode.util.pathfinder.collision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.pathfinder.DistanceSensorManager
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class CollisionDetector(private val opMode: LinearOpMode) : Subassembly {
    private val distanceSensorManager = KtHardware.get<DistanceSensorManager>(opMode)
    private var handlers: MutableMap<String, CollisionHandler> = mutableMapOf()
    private var stopRequested = false
    var collidingFront = false
        private set
    var collidingLeft = false
        private set
    var collidingRight = false
        private set

    fun startThread() = runBlocking {
        launch {
            while(opMode.opModeIsActive() && !stopRequested) {
                poll()
            }
        }
    }

    fun stopThread() { stopRequested = true }

    private fun poll() {
        if(distanceSensorManager?.frontIR?.getDistance(DistanceUnit.MM)?:100.toDouble() < 10 && !collidingFront) {
            dispatch(Collision.Front)
            collidingFront = true
        }
        if(distanceSensorManager?.leftIR?.getDistance(DistanceUnit.MM)?:100.toDouble() < 10 && !collidingLeft) {
            dispatch(Collision.Left)
            collidingLeft = true
        }
        if(distanceSensorManager?.rightIR?.getDistance(DistanceUnit.MM)?:100.toDouble() < 10 && !collidingRight) {
            dispatch(Collision.Right)
            collidingRight = true
        }
        if(distanceSensorManager?.frontIR?.getDistance(DistanceUnit.MM)?:100.toDouble() > 10 && collidingFront) {
            dispatch(Collision.EndedFront)
            collidingFront = false
        }
        if(distanceSensorManager?.leftIR?.getDistance(DistanceUnit.MM)?:100.toDouble() > 10 && collidingLeft) {
            dispatch(Collision.EndedLeft)
            collidingLeft = true
        }
        if(distanceSensorManager?.rightIR?.getDistance(DistanceUnit.MM)?:100.toDouble() > 10 && collidingRight) {
            dispatch(Collision.EndedRight)
            collidingRight = true
        }
    }

    private fun dispatch(collision: Collision) = runBlocking {
        for (handler in handlers) {
            launch { handler.value(collision) }
        }
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