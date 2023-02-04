package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import org.firstinspires.ftc.robotcore.external.Telemetry.Line
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion

object RobotConfig : BlocksOpModeCompanion() {

    fun allSubassemblies() = arrayOf(DriveBase, Arm, LinearSlide)

    fun allConnected() = allSubassemblies().filter { it.exists() }.toTypedArray()

    fun initHardwareMaps(opMode: LinearOpMode) {
        for (subassembly in allSubassemblies()) {
            CustomBlocksOpModeCompanion.initProperties(subassembly::class.java, opMode)
        }
        init()
        for(subassembly in allSubassemblies()) {
            try {
                if(subassembly.exists()) subassembly.initHardware();
            } catch (_: Throwable) {
                // no initialization required
            }
        }
    }

    fun fullRobot() {
        try { DriveBase.use(org.firstinspires.ftc.teamcode.util.drivebase.config.mecanum) } catch(_: Throwable) {}
    }

    fun programmingBoard() {
        try { DriveBase.use(org.firstinspires.ftc.teamcode.util.drivebase.config.programmingBoard) } catch(_: Throwable) {}
    }

    @ExportToBlocks(
        comment = "Initialize the robot - do this first!"
    )
    @JvmStatic
    fun init() { fullRobot() } // DO NOT CHANGE THIS

    inline fun <reified T> getHardware(name: String): T = hardwareMap.get(T::class.java, name)
}