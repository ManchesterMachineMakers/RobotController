package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower

@Autonomous(name = "Sample Autonomous")
class SampleAuto: LinearOpMode() {
  
    override fun runOpMode() {

        val driveBase = DriveBase(this)
        val vision = Vision(this)
        val follower = Follower(driveBase)

        vision.aprilTag
        follower.initialize()

        waitForStart()

        if (opModeIsActive()) {
            
        }
    }
}
