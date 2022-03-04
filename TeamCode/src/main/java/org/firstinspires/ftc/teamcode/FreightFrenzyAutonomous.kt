package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.sensors.Vision
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode
import org.firstinspires.ftc.teamcode.util.RobotConfig
import org.firstinspires.ftc.teamcode.util.Subassembly
import kotlin.math.floor

class FreightFrenzyAutonomous : MMMFreightFrenzyOpMode() {
    enum class MuffinPosition {
        Left, Middle, Right;

        companion object {
            fun fromInt(value: Int) = values().first { it.ordinal == value }
            fun of(x: Float, viewportWidth: Int): MuffinPosition {
                val sectionSize = viewportWidth / 3
                return fromInt(floor(x / sectionSize).toInt())
            }
        }
    }

    inline fun <reified T : Subassembly> getHardware(): T {
        return RobotConfig.CURRENT.getHardware(T::class.java, this)
    }

    fun log(line: String) {
        telemetry.addLine(line)
        RobotLog.i(line)
        telemetry.update()
    }

    override fun runOpMode() {
        val vision = getHardware<Vision>()
        vision.initTfod()
        vision.initVuforia()

        val blinkin = getHardware<Blinkin>()
        blinkin.autonomousDefault()

        log("Getting recognitions")
        blinkin.detecting()
        val muffinRecognitions = vision.definiteRecognitions.filter { recognition -> recognition.label == "muffin" }
        if(muffinRecognitions.isNotEmpty()) {
            val recognition = muffinRecognitions[0]
            val position = MuffinPosition.of(recognition.left, recognition.imageWidth)
            log("Recognition found at (${recognition.left}, ${recognition.top}); placement: $position")
            blinkin.detected(position)

        } else {
            log("No recognitions; not delivering")
            blinkin.autonomousDefault()
        }
    }
}