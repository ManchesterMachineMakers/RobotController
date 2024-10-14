package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log

class TankDriveBase(opMode: LinearOpMode) : Subassembly(opMode, "Tank Drive Base") {

    // get motor names from our configuration, and give them our own name
    private val leftDrive = hardwareMap.dcMotor.get("left")
    private val rightDrive = hardwareMap.dcMotor.get("right")

    init {
        // direction = FORWARD by default
        // TODO: Verify the correct directions

//        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.REVERSE

        opMode.log("MecanumDriveBase successfully initialized")
    }

    fun control(gamepad: Gamepad) {
        val leftY: Double = -gamepad.left_stick_y.toDouble()
        val rightX: Double = gamepad.right_stick_x.toDouble()

        // Calculate motor powers
        val leftPower = leftY + rightX
        val rightPower = leftY - rightX

        // Set motor powers
        leftDrive.power = leftPower
        rightDrive.power = rightPower
    }
}