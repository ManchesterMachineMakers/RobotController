package org.firstinspires.ftc.teamcode.util.tfod

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.PowerPlayAutonomous

class TimeoutObjectDetector(val tfod: TFObjectDetector) {
    data class Detection(val recognition: RecognitionMatrix, val side: String)

    /**
     * Wait until either 3 seconds have elapsed or a recognition is found, then return that recognition or null.
     */
    fun detect(self: LinearOpMode): Detection? {
        val start = System.currentTimeMillis()
        while(tfod.recognitions.size == 0 && (System.currentTimeMillis() - 3000) < start && self.opModeIsActive());
        if(tfod.recognitions.size == 0) return null
        return Detection(
            RecognitionMatrix(tfod.recognitions[0]), // Recognition matrix
            tfod.recognitions[0].label // Side
        )
    }
}