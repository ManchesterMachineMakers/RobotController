package org.firstinspires.ftc.teamcode.util.pathfinder

class Destination(val name: String, destX: Float, destY: Float) {
    //methods
    //declare variables
    private val coordinates: FloatArray = FloatArray(2)
    val x: Float
        get() = coordinates[0]
    val y: Float
        get() = coordinates[1]

    //constructor
    init {
        coordinates[0] = destX
        coordinates[1] = destY
    }
}