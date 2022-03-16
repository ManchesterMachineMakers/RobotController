package org.firstinspires.ftc.teamcode.util.pathfinder

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix

class Destination(val name: String, val destX: Float, val destY: Float, val clearanceRadius: Float) {
    //methods
    //declare variables
    // private val coordinates: FloatArray = FloatArray(2)
    // val x: Float
        // get() = coordinates[0]
    // val y: Float
        // get() = coordinates[1]
    val matrix: OpenGLMatrix
        get() = OpenGLMatrix.translation(destX, destY, 0F)

    // constructor
    // init {
        // coordinates[0] = destX
        // coordinates[1] = destY
    // }
}