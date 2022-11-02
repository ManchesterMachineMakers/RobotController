package org.firstinspires.ftc.teamcode.util.tfod

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.tfod.Recognition

class RecognitionMatrix(val recognition: Recognition) {
    val position = OpenGLMatrix.translation(
        (recognition.left + recognition.right) / 2, // Average X
        (recognition.top + recognition.bottom) / 2, // Average Y
        0F // TF is in 2D
    )
}