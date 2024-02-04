package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

fun DcMotor.config(direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD, mode: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    this.direction = direction
    this.mode = mode
}