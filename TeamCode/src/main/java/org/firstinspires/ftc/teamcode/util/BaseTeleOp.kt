package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.subassemblies.DriveBase

class BaseTeleOp(var arm: BaseArm, var driveBase: DriveBase) {
    fun init() {
        // Update statuses
        arm.init()

        // Update telemetry
        arm.telemetry()
    }

    fun start() {
        arm.start()
    }

    fun loop() {
        // Update statuses
        arm.currentStatus = "looping"

        // Loops
        driveBase.loop()
        arm.loop()

        // Update telemetry
        driveBase.telemetry()
        arm.telemetry()
    }
}