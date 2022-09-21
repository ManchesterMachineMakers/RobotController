package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.OpModeBase
import org.firstinspires.ftc.teamcode.util.drivebase.DriveBase

@TeleOp(name = "Basic TeleOp")
class BasicTeleOp : OpModeBase() {
    override fun run() {
        waitForStart()
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                if(gamepad1.x) {
                    DriveBase.go(DriveBase.TravelDirection.forward, DriveBase.DriveSpeed.SLOW)
                } else {
                    DriveBase.stop()
                }
            }
        }
    }

}