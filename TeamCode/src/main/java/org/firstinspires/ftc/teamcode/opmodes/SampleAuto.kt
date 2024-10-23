package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Sample Autonomous")
class EmptyAuto: LinearOpMode() {
  
    override fun runOpMode() {

        val driveBase = DriveBase(this)
        val vision = Vision(this)
        val follower = Follower(hardwareMap)

        vision.aprilTag()
        follower.initialize()

        waitForStart()

        if (opModeIsActive()) {
            
        }
    }
}
