package org.firstinspires.ftc.teamcode.util.drivebase.config

import org.firstinspires.ftc.teamcode.DriveBase.TravelDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.DriveBase

val programmingBoard = DriveBase.Configuration(
        arrayOf("left_front"),
        hashMapOf(
            TravelDirection.forward to arrayOf(DcMotorSimple.Direction.FORWARD),
            TravelDirection.reverse to arrayOf(DcMotorSimple.Direction.REVERSE),
            TravelDirection.pivotLeft to arrayOf(DcMotorSimple.Direction.REVERSE),
            TravelDirection.pivotRight to arrayOf(DcMotorSimple.Direction.FORWARD),
            TravelDirection.strafeLeft to arrayOf(DcMotorSimple.Direction.FORWARD),
            TravelDirection.strafeLeftBackward to arrayOf(DcMotorSimple.Direction.REVERSE),
            TravelDirection.strafeLeftForward to arrayOf(DcMotorSimple.Direction.FORWARD),
            TravelDirection.strafeRight to arrayOf(DcMotorSimple.Direction.REVERSE),
            TravelDirection.strafeRightBackward to arrayOf(DcMotorSimple.Direction.FORWARD),
            TravelDirection.strafeRightForward to arrayOf(DcMotorSimple.Direction.REVERSE),
            TravelDirection.pitch to arrayOf(DcMotorSimple.Direction.FORWARD),
        )
)