/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

import java.io.File;

import station.State;
import station.util.Persist;

/**
 * {@link DeliveryCalibration} recalibrates the slide motor, chute servos, and door servo.
 */
@TeleOp(name = "Calibration: Delivery - Resets on Init", group = "Calibration")
public class DeliveryCalibration extends MMMFreightFrenzyOpMode
    {
        int motorPosition = 0;
        double chuteServoLeftPosition = 0;
        double chuteServoRightPosition = 0;
        double doorServoPosition = 0;

        Telemetry.Line telemetrySlidePositions;
        Telemetry.Line telemetryChuteServoPositions;
        Telemetry.Line telemetryDoorServoPositions;

        Telemetry.Line telemetryMotorPosition;
        Telemetry.Line telemetryChuteServosPosition;
        Telemetry.Line telemetryDoorServoPosition;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override public void runOpMode() {
        File cfgFile = AppUtil.getInstance().getSettingsFile(RobotConfig.CURRENT.getValue("deliveryCalibrationFile"));
        cfgFile.delete();
        this.initHardware();

        telemetry.log().setCapacity(30);
        telemetry.log().add("");
        telemetry.log().add("When sufficient calibration has been reached,");
        telemetry.log().add("press the 'PS' button to write the current");
        telemetry.log().add("calibration data to a file.");
        telemetry.log().add("");
        telemetry.log().add("A - set slide HOME position");
        telemetry.log().add("X - set slide LOW position");
        telemetry.log().add("Y - set slide MID position");
        telemetry.log().add("B - set slide HIGH position");
        telemetry.log().add("right stick X - adjust slide position");
        telemetry.log().add("");
        telemetry.log().add("D-pad left - set door CLOSED position");
        telemetry.log().add("D-pad right - set door OPEN position");
        telemetry.log().add("left stick X - adjust door position");
        telemetry.log().add("");
        telemetry.log().add("D-pad down - set chute COMPACT position");
        telemetry.log().add("D-pad up - set chute OPEN position");
        telemetry.log().add("left stick Y - adjust left chute servo position");
        telemetry.log().add("right stick Y - adjust right chute servo position");
        telemetry.log().add("");

        composeTelemetry();
        telemetry.log().add("Waiting for start...");

        // Wait until we're told to go
        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        telemetry.log().add("...started...");

        while (opModeIsActive()) {

            controller();

            if (this.delivery.gamepad.ps) {
                // Save the calibration data to a file.
                String filename = RobotConfig.CURRENT.getValue("deliveryCalibrationFile");
                File file = AppUtil.getInstance().getSettingsFile(filename);
                new Persist<>(State.immutable(Delivery.state)).writeToFile(file.getAbsolutePath());
                telemetry.log().add("saved to '%s'", filename);

                // Wait for the button to be released
                while (this.delivery.gamepad.ps) {
                    telemetry.update();
                    idle();
                }
            }

            telemetry.update();
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // that we will then display in separate lines.
        telemetry.addAction(() -> {
            motorPosition = delivery.motor.getCurrentPosition();
            chuteServoLeftPosition = delivery.chuteServoLeft.getPosition();
            chuteServoRightPosition = delivery.chuteServoRight.getPosition();
            doorServoPosition = delivery.doorServo.getPosition();
        });

        telemetryMotorPosition = telemetry.addLine("Current Slide Position");
        telemetryChuteServosPosition = telemetry.addLine("Current Chute Servo Positions");
        telemetryDoorServoPosition = telemetry.addLine("Current Door Servo Position");

        telemetryMotorPosition.addData("Motor", motorPosition);
        telemetryChuteServosPosition.addData("Left", chuteServoLeftPosition )
            .addData("Right", chuteServoRightPosition );
        telemetryDoorServoPosition.addData( "Door", doorServoPosition );

        telemetrySlidePositions = telemetry.addLine("Slide Positions");
        telemetryChuteServoPositions = telemetry.addLine("Chute Servo Positions");
        telemetryDoorServoPositions = telemetry.addLine("Door Servo Positions");

        telemetrySlidePositions.addData("Home", Delivery.state.slideHomePosition)
                .addData("Low", Delivery.state.slideLowPosition)
                .addData("Mid", Delivery.state.slideMidPosition)
                .addData("High", Delivery.state.slideHighPosition);

        telemetryChuteServoPositions.addData("Left Compact", Delivery.state.chuteServoLeftCompactPosition)
                .addData("Left Open", Delivery.state.chuteServoLeftOpenPosition)
                .addData("Right Compact", Delivery.state.chuteServoRightCompactPosition)
                .addData("Right Open", Delivery.state.chuteServoRightOpenPosition);

        telemetryDoorServoPositions.addData("Closed", Delivery.state.doorServoClosedPosition)
                .addData("Open", Delivery.state.doorServoOpenPosition);
    }

        /**
         *         Default GamePad Controls
         *         A - “home” position: chute at 30 degrees, resting on drivebase
         *         X - level 1 delivery
         *         Y - level 2 delivery
         *         B - level 3 delivery
         *         D-pad up/down: override/precision control of chute height
         *         D-pad left: close door
         *         D-pad right: open door
         *         back - toggle chute to compact or open position
         *
         *         telemetry.log().add("A - set slide HOME position");
         *         telemetry.log().add("X - set slide LOW position");
         *         telemetry.log().add("Y - set slide MID position");
         *         telemetry.log().add("B - set slide HIGH position");
         *         telemetry.log().add("right stick X - adjust slide position");
         *         telemetry.log().add("");
         *         telemetry.log().add("D-pad left - set door CLOSED position");
         *         telemetry.log().add("D-pad right - set door OPEN position");
         *         telemetry.log().add("left stick X - adjust door position");
         *         telemetry.log().add("");
         *         telemetry.log().add("D-pad down - set chute COMPACT position");
         *         telemetry.log().add("D-pad up - set chute OPEN position");
         *         telemetry.log().add("left stick Y - adjust left chute servo position");
         *         telemetry.log().add("right stick Y - adjust right chute servo position");
         */
        public void controller() {
            // set the slide height
            if (!this.delivery.motor.isBusy()) {
                if (this.delivery.gamepad.a) {
                    Delivery.state.slideHomePosition = motorPosition;
                    // Wait for the button to be released
                    while (this.delivery.gamepad.a) {
                        telemetry.update();
                        idle();
                    }
                } else if (this.delivery.gamepad.x) {
                    Delivery.state.slideLowPosition = motorPosition;
                    // Wait for the button to be released
                    while (this.delivery.gamepad.x) {
                        telemetry.update();
                        idle();
                    }
                } else if (this.delivery.gamepad.y) {
                    Delivery.state.slideMidPosition = motorPosition;
                    // Wait for the button to be released
                    while (this.delivery.gamepad.y) {
                        telemetry.update();
                        idle();
                    }
                } else if (this.delivery.gamepad.b) {
                    Delivery.state.slideHighPosition = motorPosition;
                    // Wait for the button to be released
                    while (this.delivery.gamepad.b) {
                        telemetry.update();
                        idle();
                    }
                } else if (this.delivery.gamepad.right_stick_x != 0) {
                    delivery.incrementSlideUp();
                    while(delivery.gamepad.right_stick_x != 0 || delivery.motor.isBusy()) {
                        telemetry.update();
                        idle();
                    }
                }
            }
            // open and close door with left and right dpad buttons, use them to set calibration
            // adjust position of servo with left and right triggers
            else if (delivery.gamepad.left_stick_x > 0) {
                delivery.doorServo.setPosition(delivery.doorServo.getPosition() + 0.1);
                while(delivery.gamepad.left_stick_x != 0) {
                    telemetry.update();
                    idle();
                }

            } else if (delivery.gamepad.left_stick_x < 0) {
                delivery.doorServo.setPosition(delivery.doorServo.getPosition() - 0.1);
                while(delivery.gamepad.left_stick_x != 0) {
                    telemetry.update();
                    idle();
                }

            } else if (delivery.gamepad.dpad_left) {
                Delivery.state.doorServoClosedPosition = doorServoPosition;
            } else if (delivery.gamepad.dpad_right) {
                Delivery.state.doorServoOpenPosition = doorServoPosition;
            }

            // set the servo positions with the left and right sticks
            // push left for compact, right for open
            else if (delivery.gamepad.left_stick_y != 0) {
                delivery.chuteServoLeft.setPosition(delivery.chuteServoLeft.getPosition() + (delivery.gamepad.left_stick_y * 0.1));
            } else if (delivery.gamepad.right_stick_y != 0) {
                delivery.chuteServoRight.setPosition(delivery.chuteServoRight.getPosition() - (delivery.gamepad.right_stick_y * 0.1));
            } else if (delivery.gamepad.dpad_up) {
                Delivery.state.chuteServoLeftOpenPosition = chuteServoLeftPosition;
                Delivery.state.chuteServoRightOpenPosition = chuteServoRightPosition;
            } else if (delivery.gamepad.dpad_down) {
                Delivery.state.chuteServoLeftCompactPosition = chuteServoLeftPosition;
                Delivery.state.chuteServoRightCompactPosition = chuteServoRightPosition;
            }

        }
}