package org.firstinspires.ftc.teamcode.util.pathfinder

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.drivebase.DriveBase
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly

class Pathfinder(private val opMode: LinearOpMode) : Subassembly {
    val localization = KtHardware.get<Localization>(opMode)
    val driveBase = KtHardware.get<DriveBase>(opMode)

    class NoPositionError : Error("Could not get robot position")

    fun pivotTo(targetAngle: Double, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) {
        val currentAngle = localization.imu.orientation.psi
        val direction = if(targetAngle < currentAngle) DriveBase.TravelDirection.pivotRight else DriveBase.TravelDirection.pivotLeft
        driveBase.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        driveBase.go(direction, speed)
        while(localization.imu.orientation.psi != targetAngle && driveBase.isBusy && opMode.opModeIsActive()) opMode.idle()
        driveBase.stop()
    }

    fun runTo(targetX: Float, targetY: Float, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) {
        val location = localization.getRobotLocation() ?: throw NoPositionError()
        val path = Path(location[0, 0], location[1, 0], targetX, targetY, localization.imu.orientation.psi)
        // pivot
        pivotTo(path.heading)
        // run
        driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER)
        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        driveBase.go(
                if(path.direction < 0)
                    DriveBase.TravelDirection.reverse
                else
                    DriveBase.TravelDirection.forward,
                driveBase.getDriveSpeedPower(speed),
                Values.getTicks(path.distance)
        )
    }

    fun runTo(target: Destination, speed: DriveBase.DriveSpeed = DriveBase.DriveSpeed.SLOW) {
        runTo(target.x, target.y, speed)
    }
}