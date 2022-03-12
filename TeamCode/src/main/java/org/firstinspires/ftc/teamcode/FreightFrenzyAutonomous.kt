package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.sensors.Vision
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin
import org.firstinspires.ftc.teamcode.subassemblies.Delivery
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode
import org.firstinspires.ftc.teamcode.util.RobotConfig
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.pathfinder.FieldDestinations2021
import org.firstinspires.ftc.teamcode.util.pathfinder.Pathfinder
import kotlin.math.floor

open class FreightFrenzyAutonomous(private val alliance: Alliance) : MMMFreightFrenzyOpMode() {
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

    inline fun <reified T : Subassembly> getHardware(): T? {
        return RobotConfig.CURRENT.getHardware(T::class.java, this)
    }

    fun log(line: String) {
        telemetry.addLine(line)
        RobotLog.i(line)
        telemetry.update()
    }

    fun slidePositionForMuffin(position: MuffinPosition) = position.ordinal + 1

    override fun runOpMode() {
        initOpMode()
        driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER)

        val startLocation = when(alliance) {
            Alliance.Blue -> FieldDestinations2021.BlueStart1
            Alliance.Red -> FieldDestinations2021.RedStart1
        }

        val vision = getHardware<Vision>()
        vision?.initVuforiaForTFOD()
        vision?.initTfod()
        vision?.activateTfod()

        val blinkin = getHardware<Blinkin>()
        blinkin?.autonomousDefault()

        val delivery = getHardware<Delivery>()
        val pathfinder = getHardware<Pathfinder>()

        waitForStart()
        log("Getting recognitions")
        blinkin?.detecting()
        val muffinRecognitions = vision?.definiteRecognitions?.filter { recognition -> recognition.label == "muffin" }
        vision?.deactivateTFOD()
        val deliverTo = if(muffinRecognitions?.isNotEmpty() == true) {
            val recognition = muffinRecognitions[0]
            val position = MuffinPosition.of(recognition.left, recognition.imageWidth)
            log("Recognition found at (${recognition.left}, ${recognition.top}); placement: $position")
            blinkin?.detected(position)
            position
        } else {
            log("No recognitions; delivering to middle")
            blinkin?.autonomousDefault()
            MuffinPosition.Middle
        }

        log("Delivering to (${deliverTo})")
        delivery?.runSlideToPosition(slidePositionForMuffin(deliverTo))

        val targetHub = when(alliance) {
            Alliance.Blue -> FieldDestinations2021.BlueHub
            Alliance.Red -> FieldDestinations2021.RedHub
        }
        pathfinder?.runTo(targetHub.destination, startLocation.destination.matrix)

        delivery?.setChuteOpenPosition()
        delivery?.setDoorOpenPosition()

        log("Parking in warehouse")
        val targetWarehouse = when(alliance) {
            Alliance.Blue -> FieldDestinations2021.BlueWarehouse
            Alliance.Red -> FieldDestinations2021.RedWarehouse
        }
        pathfinder?.runTo(targetWarehouse.destination, targetHub.destination.matrix)

        log("Closing delivery")
        delivery?.setDoorClosedPosition()
        delivery?.setChuteCompactPosition()
    }
}