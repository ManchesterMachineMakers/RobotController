package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.diagnostics.tests.*;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;

import java.lang.annotation.Annotation;
import java.util.Arrays;

public class Runner {
    public final Base[] tests = {
            new DriveBaseTest(),
            new LightingTest(),
            new DeliveryTest(),
            new IntakeTest()
            /*new RingSensorTest()*/ };
            /*new GamepadTest() }; */
    public final DiagnosticsOpMode opMode;
    private final Testable[] sel;
    private String currentTest = null;
    public Runner(Testable[] sel, DiagnosticsOpMode opMode) {
        this.sel = sel;
        this.opMode = opMode;
    }
    public void runAll() throws InterruptedException {
        log("Beginning run of all tests");
        for(Base test : tests) {
            if (!run(test)) {
                opMode.telemetry.addLine("Test failed. See log for details.");
                log("Failed to run tests; see above for details");
            }
        }
        log("Finished running tests");
    }
    public boolean run(Base test) throws InterruptedException {
        Class<?> cls = test.getClass();
        Annotation[] annotations = cls.getDeclaredAnnotations();
        currentTest = cls.getCanonicalName();
        boolean requirementsMet = true;
        for(Annotation annotation : annotations) {
            if(annotation.annotationType() == Test.class) {
                currentTest = ((Test)annotation).value();
            }
            if(annotation.annotationType() == Requires.class) {
                log("Checking requirement " + ((Requires)annotation).value().getName());
                boolean met = false;
                for (Testable provided : this.opMode.provides()) {
                    try {
                        ((Requires)annotation).value().cast(provided);
                        met = true;
                    } catch (ClassCastException ignored) {}
                }
                requirementsMet = requirementsMet && met;
            }
        }
        if(requirementsMet) {
            log("Running test: " + currentTest);
            try {
                boolean result = test.run(sel, this);
                if (!result) {
                    throw new Exception("Test failed");
                }
            } catch(Exception e) {
                log("Failed to run: " + currentTest + ", reason: " + e.getMessage());
                return false;
            }
            log("Completed successfully: " + currentTest);
            return true;
        } else {
            log("Not running test " + currentTest + ", requirements not met");
            return false;
        }
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
