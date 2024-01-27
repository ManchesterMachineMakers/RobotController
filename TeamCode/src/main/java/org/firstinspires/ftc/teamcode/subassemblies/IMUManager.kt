package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.Subassembly


class IMUManager(opMode: OpMode) : Subassembly(opMode, "IMU Manager") {
    val imuParameters = IMU.Parameters(
        RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        )
    )
    val imu = hardwareMap.get(IMU::class.java, "imu")

    init {
        imu.initialize(imuParameters)
    }

    override fun loop(gamepad: Gamepad) {
        val orientation = imu.robotYawPitchRollAngles
        val angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES))
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES))
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES))
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate)
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate)
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate)
    }
}