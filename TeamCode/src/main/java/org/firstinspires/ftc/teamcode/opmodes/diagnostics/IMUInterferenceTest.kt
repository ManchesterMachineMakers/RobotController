package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subassemblies.IMUManager

@TeleOp
class IMUInterferenceTest : LinearOpMode() {
    override fun runOpMode() {
        val imu = IMUManager(this)
        waitForStart()
        imu.imu.resetYaw()
        val telemetryLine = telemetry.addData("IMU Yaw", imu.yaw)
        while(opModeIsActive() && !isStopRequested) {
            telemetryLine.setValue(imu.yaw)
            telemetry.update()
        }
    }
}