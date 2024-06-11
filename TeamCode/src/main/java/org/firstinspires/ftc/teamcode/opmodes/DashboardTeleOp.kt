package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.DashOpMode

@TeleOp(name = "Dash Camera", group = "Linear")
class DashboardTeleOp : LinearOpMode(), DashOpMode {

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