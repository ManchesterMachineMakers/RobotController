package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
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
                return fromInt(2 - floor(x / sectionSize).toInt()) // .tob eht no sdrawkcab si aremac ehT
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

        report("Warning! Please retract the slides completely to zero before running this op mode!")
        keepTheBeat(3)
        report("Slides are now AT ZERO.  If they are not FULLY RETRACTED, you will break them!")
        keepTheBeat(3)
        report("If they are not retracted, stop this OpMode, retract the slides, and restart.")
        keepTheBeat(5)
        // Start it talking
        report("Are you ready?")
        keepTheBeat(1)

        waitForStart()
        log("Intaking preloaded freight")
        delivery?.setChuteHomePosition()
        intake?.go(DcMotorSimple.Direction.FORWARD);
        sleep(3000)
        intake?.stop()
        log("Getting recognitions")
        blinkin?.detecting()
        val muffinRecognitions = vision?.definiteRecognitions?.filter { recognition -> recognition.label == "muffin" && recognition.confidence > 0.95 }
        vision?.deactivateTFOD()
        val deliverTo = if(muffinRecognitions?.isNotEmpty() == true) {
            val recognition = muffinRecognitions[0]
            val position = MuffinPosition.of(recognition.left, recognition.imageWidth)
            log("Recognition found at (${recognition.left}, ${recognition.top}); placement: $position")
            report("There's a muffin at the ${position}! I love muffins.")
            blinkin?.detected(position)
            position
        } else {
            log("No recognitions; delivering to middle")
            report("Did not find a muffin. Too bad!")
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

        delivery?.setChuteDeliverPosition()
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
        delivery?.runSlideToPosition(Delivery.SLIDE_HOME_POSITION);
    }
}
