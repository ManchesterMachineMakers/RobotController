package org.firstinspires.ftc.teamcode.diagnostics.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.tests.Base;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public abstract class DiagnosticsOpMode extends LinearOpMode {
    private List<Class<? extends Base>> _tests = new ArrayList<>();
    public Testable[] provides() {
        return RobotConfig.CURRENT.getTestable(this);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Blinkin ledUtil = new Blinkin(hardwareMap);
        Runner runner = new Runner(this);
        tests();
        RobotLog.i("16221 Diagnostics Opmode: Initialization complete.");
        telemetry.addLine("Initialized.");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            RobotLog.i("16221 Diagnostics Opmode: Running all tests.");
            telemetry.addLine("Running tests. Please watch the log.");
            telemetry.update();
            runTests(runner);
        }
    }
    protected void test(Class<? extends Base> cls) {
        _tests.add(cls);
    }
    protected void tests() {

    }
    protected void runTests(Runner runner) throws InterruptedException {
        if(_tests.size() == 0) {
            runner.runAll();
        } else {
            for (Class<? extends Base> testCls:
                 _tests) {
                try {
                    Base test = testCls.getConstructor().newInstance();
                    if (!runner.run(test)) {
                        this.telemetry.addLine("Test failed. See log for details.");
                        runner.log("Failed to run tests; see above for details");
                    }
                } catch (Exception e) {
                    this.telemetry.addLine("Error when initializing test " + testCls.getSimpleName());
                    runner.log("Error when initializing test " + testCls.getSimpleName());
                }
                telemetry.update();
            }
        }
    }
}
