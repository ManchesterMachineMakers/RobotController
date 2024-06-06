package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.opencv.core.Scalar

@TeleOp(name = "Dash Camera", group = "Linear")
class DashboardTeleOp : LinearOpMode(), DashOpMode {

    @Config
    object RobotConstants {
        @JvmField var armSpeed: Scalar = Scalar(-1.0, -1.0, -1.0, 1.0)
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val driveBase = DriveBase(this)
        val arm = Arm(this)
        val vision = Vision(this)

        vision.visionPortal

        waitForStart()

        FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)

        while (opModeIsActive()) {
            sleep(100)
            driveBase.control(gamepad1)
        }
    }
}