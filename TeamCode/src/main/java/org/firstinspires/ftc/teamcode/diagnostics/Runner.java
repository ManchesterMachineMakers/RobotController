package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.tests.Base;
import org.firstinspires.ftc.teamcode.diagnostics.tests.DriveBaseTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.DuckyVisionTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.GamepadTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.IntakeTest;
import org.firstinspires.ftc.teamcode.diagnostics.tests.Test;
import org.firstinspires.ftc.teamcode.diagnostics.tests.DeliveryTest;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;

import java.lang.annotation.Annotation;

public class Runner {
    public final Base[] tests = {
            new DriveBaseTest()
            /*, new LightingTest()*/
            /*, new DeliveryTest()*/
            /*, new IntakeTest()*/
            , new DuckyVisionTest()
            /*, new GamepadTest() */
            };
    public final LinearOpMode opMode;
    private Selectors sel;
    private String currentTest = null;
    public Runner(Selectors sel, LinearOpMode opMode) {
        this.sel = sel;
        this.opMode = opMode;
    }
    public void runAll() throws InterruptedException {
        log("Beginning run of all tests");
        for(Base test : tests) {
            if(!run(test)) {
                opMode.telemetry.addLine("Test failed. See log for details.");
                log("Failed to run tests; see above for details");
            }
        }
        log("Finished running tests");
    }
    public boolean run(Base test) throws InterruptedException {
        Class<?> cls = test.getClass();
        Annotation[] annotations = cls.getDeclaredAnnotations();
        String name = cls.getName();
        for(Annotation annotation : annotations) {
            if(annotation.annotationType() == Test.class) {
                name = ((Test)annotation).value();
            }
        }
        log("Running test: " + name);
        this.currentTest = name;
        boolean result = test.run(sel, this);
        if(!result) {
            log("Failed to run: " + name);
            return false;
        }
        log("Completed successfully: " + name);
        return true;
    }
    public void log(String msg) {
        if(this.currentTest != null) {
            RobotLog.i("16221 Diagnostics System: " + this.currentTest + ": " + msg);
        } else {
            RobotLog.i("16221 Diagnostics System: " + msg);
        }
    }
    public boolean abort() {
        log("Aborting test");
        return false;
    }
}
