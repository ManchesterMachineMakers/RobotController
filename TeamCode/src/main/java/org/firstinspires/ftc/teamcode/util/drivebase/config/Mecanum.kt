package org.firstinspires.ftc.teamcode.util.drivebase.config

import org.firstinspires.ftc.teamcode.util.drivebase.DriveBase
import com.qualcomm.robotcore.hardware.DcMotorSimple

val mecanum = DriveBase.Configuration(
        arrayOf(
                "left_front", "right_front",
                "left_rear",  "right_rear"
        ),
        hashMapOf(
                DriveBase.TravelDirection.base to arrayOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE),
                DriveBase.TravelDirection.forward to arrayOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, ),
                DriveBase.TravelDirection.reverse to arrayOf(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, ),
                DriveBase.TravelDirection.pivotLeft to arrayOf(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, ),
                DriveBase.TravelDirection.pivotRight to arrayOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, ),
                DriveBase.TravelDirection.strafeLeft to arrayOf(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, ),
                DriveBase.TravelDirection.strafeLeftForward to arrayOf(null, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, null, ),
                DriveBase.TravelDirection.strafeLeftBackward to arrayOf(DcMotorSimple.Direction.REVERSE, null, null, DcMotorSimple.Direction.REVERSE, ),
                DriveBase.TravelDirection.strafeRight to arrayOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, ),
                DriveBase.TravelDirection.strafeRightForward to arrayOf(DcMotorSimple.Direction.FORWARD, null, null, DcMotorSimple.Direction.FORWARD, ),
                DriveBase.TravelDirection.strafeRightBackward to arrayOf(null, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, null, ),
                DriveBase.TravelDirection.pitch to arrayOf(null, null, null, null, ),
        )
)