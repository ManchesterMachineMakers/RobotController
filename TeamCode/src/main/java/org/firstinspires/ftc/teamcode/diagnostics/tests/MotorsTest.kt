package org.firstinspires.ftc.teamcode.diagnostics.tests

import org.firstinspires.ftc.teamcode.drivebase.DriveBase
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode
import java.lang.Exception

fun motorsTest(opMode: DiagnosticsOpMode) = describe<DriveBase> { driveBase ->
    val testMotors = driveBase.motors
    opMode.telemetry.isAutoClear = false
    val tRuntime = opMode.telemetry.addData("Runtime:", opMode.runtime)
    val tStatus = opMode.telemetry.addData("Status:", "* Drive Base Initialized")
    val tRunMode = opMode.telemetry.addData("Run Mode:", testMotors[0].mode)
    val tDirection = opMode.telemetry.addData("Direction:", testMotors[0].direction)
    val tPower = opMode.telemetry.addData("Power:", testMotors[0].power)
    opMode.telemetry.addLine("****")
    opMode.telemetry.addLine("Initialized.")
    opMode.telemetry.update()
    RobotLog.i("Initialized.")

    driveBase.setRunMode(RunMode.STOP_AND_RESET_ENCODER)
    tStatus!!.setValue("** Stopped and reset encoders.")
    tRunMode!!.setValue(testMotors[0].mode)
    tDirection!!.setValue(testMotors[0].direction)
    tPower!!.setValue(testMotors[0].power)
    opMode.telemetry.addLine("Stopped and reset encoders.")
    RobotLog.i("Stopped and reset encoders")
    opMode.telemetry.update()
    // test each motor //
    val testPower = 0.2
    var result: Boolean
    try {
        for (i in testMotors.indices) {
            tStatus.setValue("*** " + i + ") " + testMotors[i].deviceName)
            opMode.telemetry.addLine("Testing Motor " + i + ") " + testMotors[i].deviceName)
            opMode.telemetry.update()
            RobotLog.i("Testing Motor " + i + ") " + testMotors[i].deviceName)
            for (rm in RunMode.values()) {
                //if (!opModeIsActive()) { break; }
                opMode.telemetry.addLine("Testing run mode " + rm.name)
                opMode.telemetry.update()
                RobotLog.i("Testing run mode " + rm.name)
                testMotors[i].mode = rm
                assert(testMotors[i].mode == rm)
                for (direction in DcMotorSimple.Direction.values()) {
                    //if (!opModeIsActive()) { break; }
                    testMotors[i].direction = direction
                    assert(testMotors[i].direction == direction)
                    if (rm == RunMode.RUN_TO_POSITION) {
                        testMotors[i].mode = RunMode.RUN_USING_ENCODER
                        // set it to one revolution
                        testMotors[i].targetPosition = driveBase.motorEncoderEventsPerRotation.toInt()
                        assert(testMotors[i].targetPosition == driveBase.motorEncoderEventsPerRotation.toInt())
                        (testMotors[i] as DcMotorEx).targetPositionTolerance = 50
                        assert((testMotors[i] as DcMotorEx).targetPositionTolerance == 50)
                        testMotors[i].mode = rm
                        assert(testMotors[i].mode == rm)
                    }
                    val timeout = opMode.runtime + 1
                    testMotors[i].power = testPower
                    assert(testMotors[i].power == testPower)
                    tRunMode.setValue(testMotors[i].mode)
                    tDirection.setValue(testMotors[i].direction)
                    tPower.setValue(testMotors[i].power)
                    opMode.telemetry.update()
                    RobotLog.i("Direction: " + testMotors[i].direction + " Power: " + testMotors[i].power)
                    while (driveBase.isBusy &&  //opModeIsActive() &&
                            (opMode.runtime < timeout || testMotors[i].mode == RunMode.RUN_TO_POSITION)) {
                        tRuntime!!.setValue(opMode.runtime)
                        opMode.telemetry.update()
                        opMode.idle()
                    }
                    driveBase.stop()
                    assert(testMotors[i].power == 0.0)
                    if (testMotors[i].mode == RunMode.RUN_TO_POSITION) {
                        assert(testMotors[i].currentPosition == driveBase.motorEncoderEventsPerRotation.toInt())
                        testMotors[i].mode = RunMode.STOP_AND_RESET_ENCODER
                        assert(testMotors[i].currentPosition == 0)
                        testMotors[i].mode = RunMode.RUN_USING_ENCODER
                    }
                }
            }
        }
        result = true
    } catch (ex: Exception) {
        RobotLog.e(ex.message)
        result = false
    } finally {
        driveBase.cleanup()
    }

    result.expect("Failed to complete test")
}