package org.firstinspires.ftc.teamcode.diagnostics.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.DriveBase;

@Test("Motors Test")
@Requires(DriveBase.class)
public class MotorsTest implements Base {

    DriveBase driveBase;
    Telemetry.Item tRuntime;
    Telemetry.Item tRunMode;
    Telemetry.Item tDirection;
    Telemetry.Item tPower;
    Telemetry.Item tStatus;

    DcMotor[] testMotors;


    public void init(OpMode opMode, DriveBase driveBase) {
        testMotors = driveBase.getMotors();
        opMode.telemetry.setAutoClear(false);
        tRuntime = opMode.telemetry.addData("Runtime:", opMode.getRuntime());
        tStatus = opMode.telemetry.addData("Status:", "* Drive Base Initialized");
        tRunMode = opMode.telemetry.addData("Run Mode:", testMotors[0].getMode());
        tDirection = opMode.telemetry.addData("Direction:", testMotors[0].getDirection());
        tPower =  opMode.telemetry.addData("Power:", testMotors[0].getPower());
        opMode.telemetry.addLine("****");
        opMode.telemetry.addLine("Initialized.");
        opMode.telemetry.update();

        RobotLog.i("Initialized.");
    }

    @Override
    public boolean run(Testable[] sel, Runner runner) throws Exception {
        driveBase = Testable.getOrDie(sel, DriveBase.class);
        init(runner.opMode, driveBase);

        driveBase.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tStatus.setValue("** Stopped and reset encoders.");
        tRunMode.setValue(testMotors[0].getMode());
        tDirection.setValue(testMotors[0].getDirection());
        tPower.setValue(testMotors[0].getPower());
        runner.opMode.telemetry.addLine("Stopped and reset encoders.");
        RobotLog.i("Stopped and reset encoders");

        runner.opMode.telemetry.update();
        /** test each motor **/

        double testPower = 0.2;

        boolean result = false;
        try {

            for (int i = 0; i < testMotors.length; i++) {
                tStatus.setValue("*** " + String.valueOf(i) + ") " + testMotors[i].getDeviceName());
                runner.opMode.telemetry.addLine("Testing Motor " + String.valueOf(i) + ") " +  testMotors[i].getDeviceName());
                runner.opMode.telemetry.update();
                RobotLog.i("Testing Motor " + String.valueOf(i) + ") " +  testMotors[i].getDeviceName());

                for (DcMotor.RunMode rm:
                        DcMotor.RunMode.values()) {
                    //if (!opModeIsActive()) { break; }
                    runner.opMode.telemetry.addLine("Testing run mode " + rm.name());
                    runner.opMode.telemetry.update();
                    RobotLog.i("Testing run mode " + rm.name());


                    testMotors[i].setMode(rm);
                    assert(testMotors[i].getMode() == rm);

                    for (DcMotorSimple.Direction direction:
                            DcMotorSimple.Direction.values()) {
                        //if (!opModeIsActive()) { break; }

                        testMotors[i].setDirection(direction);
                        assert(testMotors[i].getDirection() == direction);

                        if (rm == DcMotor.RunMode.RUN_TO_POSITION) {
                            testMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            // set it to one revolution
                            testMotors[i].setTargetPosition((int) driveBase.motorEncoderEventsPerRotation);
                            assert(testMotors[i].getTargetPosition() == (int) driveBase.motorEncoderEventsPerRotation);

                            ((DcMotorEx)testMotors[i]).setTargetPositionTolerance(50);
                            assert(((DcMotorEx)testMotors[i]).getTargetPositionTolerance() == 50);

                            testMotors[i].setMode(rm);
                            assert(testMotors[i].getMode() == rm);
                        }

                        double timeout = runner.opMode.getRuntime() + 1;
                        testMotors[i].setPower(testPower);
                        assert(testMotors[i].getPower() == testPower);

                        tRunMode.setValue(testMotors[i].getMode());
                        tDirection.setValue(testMotors[i].getDirection());
                        tPower.setValue(testMotors[i].getPower());
                        runner.opMode.telemetry.update();
                        RobotLog.i("Direction: " + testMotors[i].getDirection() + " Power: " + String.valueOf(testMotors[i].getPower()));

                        while(driveBase.isBusy() && //opModeIsActive() &&
                                (runner.opMode.getRuntime() < timeout || testMotors[i].getMode() == DcMotor.RunMode.RUN_TO_POSITION)) {
                            tRuntime.setValue(runner.opMode.getRuntime());
                            runner.opMode.telemetry.update();
                            runner.opMode.idle();
                        }
                        driveBase.stop();
                        assert(testMotors[i].getPower() == 0);

                        if (testMotors[i].getMode() ==  DcMotor.RunMode.RUN_TO_POSITION) {
                            assert(testMotors[i].getCurrentPosition() == (int) driveBase.motorEncoderEventsPerRotation);

                            testMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            assert(testMotors[i].getCurrentPosition() == 0);

                            testMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                    }
                }
            }
            result = true;
        } catch (Exception ex) {
            RobotLog.e(ex.getMessage());
            result = false;
        } finally {
            driveBase.cleanup();
        }
        return result;
    }

}
