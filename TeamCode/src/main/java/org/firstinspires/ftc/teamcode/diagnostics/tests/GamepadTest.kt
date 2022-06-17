package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad

/*** Controls ***
 *
 * Match TeleOp
 * Gamepad 1 - Drivebase steering/speed control with joysticks
 * Gamepad 2 - Intake and Delivery controls
 * Intake
 * RB - Take in
 * RT - Take in (slow)
 * LB - Push out
 * LT - Push out (slow)
 * Delivery
 * A - “home” position: chute at 30 degrees, resting on drivebase
 * X - level 1 delivery
 * Y - level 2 delivery
 * B - level 3 delivery
 * D-pad up/down: override/precision control of chute height
 *
 * Diagnostics mode
 * Assign all controls to Gamepad 1
 * Add control of chute angle to D-pad left/right
 *
 */
fun gamepadTest(opMode: DiagnosticsOpMode) = describe<Gamepad> { gamepad ->
    val gmp1 = gamepad[1]

    it("works") {
        val lx = opMode.telemetry.addData("Left Stick X", gmp1.left_stick_x)
        val ly = opMode.telemetry.addData("Left Stick Y", gmp1.left_stick_y)

        val rx = opMode.telemetry.addData("Right Stick X", gmp1.right_stick_x)
        val ry = opMode.telemetry.addData("Right Stick Y", gmp1.right_stick_y)

        val lb = opMode.telemetry.addData("Left Stick Button", gmp1.left_stick_button)
        val rb = opMode.telemetry.addData("Right Stick Button", gmp1.right_stick_button)

        val padl = opMode.telemetry.addData("Left Pad", gmp1.dpad_left)
        val padu = opMode.telemetry.addData("Up Pad", gmp1.dpad_up)
        val padr = opMode.telemetry.addData("Right Pad", gmp1.dpad_right)
        val padd = opMode.telemetry.addData("Down Pad", gmp1.dpad_down)

        val lTrig = opMode.telemetry.addData("Left Trigger", gmp1.left_trigger)
        val rTrig = opMode.telemetry.addData("Right Trigger", gmp1.right_trigger)

        val lBump = opMode.telemetry.addData("Left Bumper", gmp1.left_bumper)
        val rBump = opMode.telemetry.addData("Right Bumper", gmp1.right_bumper)

        val aButton = opMode.telemetry.addData("A", gmp1.a)
        val bButton = opMode.telemetry.addData("B", gmp1.b)
        val xButton = opMode.telemetry.addData("X", gmp1.x)
        val yButton = opMode.telemetry.addData("Y", gmp1.y)

        val backButton = opMode.telemetry.addData("Back", gmp1.back)

        val startButton = opMode.telemetry.addData("Start", gmp1.start)

        while (!opMode.gamepad1.ps && opMode.opModeIsActive()) {
            lx.setValue(gmp1.left_stick_x)
            ly.setValue(gmp1.left_stick_y)

            rx.setValue(gmp1.right_stick_x)
            ry.setValue(gmp1.right_stick_y)

            lb.setValue(gmp1.left_stick_button)

            rb.setValue(gmp1.right_stick_button)

            padl.setValue(gmp1.dpad_left)
            padu.setValue(gmp1.dpad_up)
            padr.setValue(gmp1.dpad_right)
            padd.setValue(gmp1.dpad_down)

            lTrig.setValue(gmp1.left_trigger)
            rTrig.setValue(gmp1.right_trigger)

            lBump.setValue(gmp1.left_bumper)
            rBump.setValue(gmp1.right_bumper)

            aButton.setValue(gmp1.a)
            bButton.setValue(gmp1.b)
            xButton.setValue(gmp1.x)
            yButton.setValue(gmp1.y)

            backButton.setValue(gmp1.back)

            startButton.setValue(gmp1.start)

            opMode.telemetry.update()
        }
    }
    it("can rumble") {
        opMode.telemetry.addLine("Ready to Rumble")
        opMode.telemetry.update()

        gmp1.rumble(100)

        opMode.telemetry.addLine("Rumble complete.")
        opMode.telemetry.update()
    }
}