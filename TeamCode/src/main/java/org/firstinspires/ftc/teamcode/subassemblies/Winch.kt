package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.curveJoystickVal

class Winch(opMode: OpMode): Subassembly(opMode, "Winch") {

    val winchMotor = hardwareMap.dcMotor.get("winch")

    init {
        winchMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER // NOTE: encoders on winch motor are damaged
        winchMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        winchMotor.direction = DcMotorSimple.Direction.FORWARD // TODO: double check this is the optimal direction

        telemetry.addData(">", "Winch Subassembly Ready")
    }

    fun control(gamepad: Gamepad) { winchMotor.power = curveJoystickVal(gamepad.right_stick_y) }
}