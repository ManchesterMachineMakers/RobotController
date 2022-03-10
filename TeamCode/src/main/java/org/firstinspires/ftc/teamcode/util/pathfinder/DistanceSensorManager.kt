package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly

class DistanceSensorManager(private val opMode: LinearOpMode) : Subassembly {
    val rightIR = opMode.hardwareMap[DistanceSensor::class.java, KtHardware.name("range_Right")]
    val leftIR = opMode.hardwareMap[DistanceSensor::class.java, KtHardware.name("range_Right")]
    val frontIR = opMode.hardwareMap[DistanceSensor::class.java, KtHardware.name("range_Front")]
}