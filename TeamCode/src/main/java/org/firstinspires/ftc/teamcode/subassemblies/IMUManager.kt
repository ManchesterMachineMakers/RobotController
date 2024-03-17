package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log

class IMUManager(opMode: LinearOpMode) : Subassembly(opMode, "IMU Manager") {
    val imuParameters =
            IMU.Parameters(
                    RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
            )
    val imu = hardwareMap.get(IMU::class.java, "imu")

    init {
        imu.initialize(imuParameters)
        opMode.log("IMUManager successfully initialized")
    }

    override fun telemetry() {
        val orientation = imu.robotYawPitchRollAngles
        val angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)

        super.telemetry()
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES))
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES))
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES))
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate)
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate)
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate)
    }
}
