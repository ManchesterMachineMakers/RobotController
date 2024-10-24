package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Empty Autonomous")
class EmptyAuto: LinearOpMode() {

    override fun runOpMode() {
        waitForStart()

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                sleep(100)
            }
        }
    }
}