package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.subassemblies.Delivery
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import org.firstinspires.ftc.teamcode.subassemblies.Gamepad

fun deliveryTest(opMode: DiagnosticsOpMode) = describe<Delivery> { delivery ->
    it("is retracted") {
        opMode.telemetry.speak("Important! Please retract the slides completely to zero before running this op mode!")
        Thread.sleep(3000)
        opMode.telemetry.speak("Repeat: Please retract the slides completely to zero before running this op mode!")
        Thread.sleep(3000)
        opMode.telemetry.addLine("Slides are now AT ZERO.  If they are not FULLY RETRACTED, you will break them!  If they are not retracted, stop this OpMode, retract the slides, and restart.")
        opMode.telemetry.update()
        Thread.sleep(3000)
    }

    it("can run itself") {
        log("Home")
        delivery.runSlideToPosition(Delivery.SLIDE_HOME_POSITION)

        log("Low")
        delivery.runSlideToPosition(Delivery.SLIDE_LOW_POSITION)

        Thread.sleep(1000)

        log("Mid")
        delivery.runSlideToPosition(Delivery.SLIDE_MID_POSITION)

        Thread.sleep(1000)

        log("High")
        delivery.runSlideToPosition(Delivery.SLIDE_HIGH_POSITION)

        Thread.sleep(1000)

        log("Cap")
        delivery.runSlideToPosition(Delivery.SLIDE_CAP_POSITION)

        Thread.sleep(1000)

        log("Open")
        delivery.setDoorOpenPosition()

        Thread.sleep(1000)

        log("Closed")
        delivery.setDoorClosedPosition()

        Thread.sleep(1000)

        log("Unfold")
        delivery.setChuteDeliverPosition()

        Thread.sleep(1000)

        log("Fold")
        delivery.setChuteCompactPosition()

        Thread.sleep(1000)

        log("Home")
        delivery.runSlideToPosition(0)

        Thread.sleep(1000)
    }

    it("can be controlled") {
        log("Testing Controls")

        val motorPos = opMode.telemetry.addData("Slide Motor Position", delivery.motor.currentPosition)
        val servoLeftPos = opMode.telemetry.addData("Left Servo Position", delivery.chuteServoLeft.position)
        val servoRightPos = opMode.telemetry.addData("Right Servo Position", delivery.chuteServoRight.position)
        val servoDoorPos = opMode.telemetry.addData("Door Servo Position", delivery.doorServo.position)

        val gmp1 = require<Gamepad>()[1]

        val padl = opMode.telemetry.addData("Left Pad", gmp1.dpad_left)
        val padu = opMode.telemetry.addData("Up Pad", gmp1.dpad_up)
        val padr = opMode.telemetry.addData("Right Pad", gmp1.dpad_right)
        val padd = opMode.telemetry.addData("Down Pad", gmp1.dpad_down)

        val aButton = opMode.telemetry.addData("A", gmp1.a)
        val bButton = opMode.telemetry.addData("B", gmp1.b)
        val xButton = opMode.telemetry.addData("X", gmp1.x)
        val yButton = opMode.telemetry.addData("Y", gmp1.y)

        val backButton = opMode.telemetry.addData("Back", gmp1.back)

        opMode.telemetry.update()

        while (!opMode.gamepad1.ps && opMode.opModeIsActive()) {
            delivery.controller(opMode)

            motorPos.setValue(delivery.motor.currentPosition)
            servoLeftPos.setValue(delivery.chuteServoLeft.position)
            servoRightPos.setValue(delivery.chuteServoRight.position)
            servoDoorPos.setValue(delivery.doorServo.position)

            padl.setValue(gmp1.dpad_left)
            padu.setValue(gmp1.dpad_up)
            padr.setValue(gmp1.dpad_right)
            padd.setValue(gmp1.dpad_down)

            aButton.setValue(gmp1.a)
            bButton.setValue(gmp1.b)
            xButton.setValue(gmp1.x)
            yButton.setValue(gmp1.y)

            backButton.setValue(gmp1.back)

            opMode.telemetry.update()
        }
    }
}