package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.powerCurve

class Winch(opMode: LinearOpMode): Subassembly(opMode, "Winch") {

    val winchMotor = hardwareMap.dcMotor.get("winch")

    init {
        winchMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER // NOTE: encoders on winch motor are damaged
        winchMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        winchMotor.direction = DcMotorSimple.Direction.FORWARD // TODO: double check this is the optimal direction

        opMode.log("Winch successfully initialized")
    }

    fun control(joystick: Float) { winchMotor.power = powerCurve(joystick.toDouble()) }
}