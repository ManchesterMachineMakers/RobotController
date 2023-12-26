package org.firstinspires.ftc.teamcode.util.bases

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase

class BaseArmTeleOp(val opMode: OpMode, private val driveBase: DriveBase, private val arm: BaseArm) {
    fun init() {
        // Update statuses
        driveBase.status = "initializing"
        arm.status = "initializing"
    }

    fun start() {
        arm.status = "starting"
        arm.start()
    }

    fun loop() {
        // Update statuses
        driveBase.status = "looping"
        arm.status = "looping"

        // Loops
        driveBase.loop()
        arm.loop()
    }

    fun telemetry() {
        driveBase.telemetry()
        arm.telemetry()
        opMode.telemetry.update()
    }
}