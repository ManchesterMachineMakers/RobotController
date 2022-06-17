package org.firstinspires.ftc.teamcode.diagnostics.tests

import org.firstinspires.ftc.teamcode.drivebase.DriveBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode

fun driveBaseTest(opMode: DiagnosticsOpMode) = describe<DriveBase> { driveBase ->
    val power = 0.2
    it("can move each motor individually") {
        log("Testing each motor")
        driveBase.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        val motors = driveBase.motors
        for (motor in motors) {
            log("Motor: " + motor.deviceName)
            motor.power = power
            opMode.sleep(500)
            motor.power = 0 - power
            opMode.sleep(500)
            motor.power = 0.0
        }
    }

    it("can move in a coordinated fashion") {
        log("Testing coordinated motion")

        log("Moving forward 500 ticks w/ encoder")
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        driveBase.go(DriveBase.TravelDirection.forward, power, 500, 50)

        while (driveBase.isBusy);

        log("Moving backward 500 ticks w/ encoder")
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        driveBase.go(DriveBase.TravelDirection.reverse, power, 500, 50)

        while (driveBase.isBusy);

        log("Strafing left 500 ticks w/ encoder")
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        driveBase.go(DriveBase.TravelDirection.strafeLeft, power, 500, 50)

        while (driveBase.isBusy);

        log("Strafing right 500 ticks w/ encoder")
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        driveBase.go(DriveBase.TravelDirection.strafeRight, power, 500, 50)

        while (driveBase.isBusy);
    }

//        runner.log("Pitching to 30 degrees");
//        driveBase.pitch(30);
//        Thread.sleep(1000);
//        runner.log("Returning to straight");
//        driveBase.pitch(0);
//        Thread.sleep(1000);
}