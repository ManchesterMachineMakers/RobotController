package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

fun DcMotor.config(direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD) {
    this.mode = DcMotor.RunMode.RUN_USING_ENCODER
    this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    this.direction = direction
}