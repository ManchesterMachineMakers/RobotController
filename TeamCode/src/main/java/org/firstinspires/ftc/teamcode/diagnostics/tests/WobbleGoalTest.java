package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.subassemblies.WobbleGoalGrabber;

@Test("Wobble Goal Grabber Test")
public class WobbleGoalTest implements Base {

    public boolean run(Selectors sel, Runner runner) throws InterruptedException {
        WobbleGoalGrabber wgg = sel.wggSelector().get();
        runner.log("Up");
        wgg.up();
        Thread.sleep(1000);
        runner.log("Grab");
        wgg.grab();
        Thread.sleep(1000);
        runner.log("Release");
        wgg.release();
        Thread.sleep(1000);
        runner.log("Down");
        wgg.down();
        Thread.sleep(1000);
        runner.log("Up");
        wgg.up();
        Thread.sleep(1000);
        runner.log("Grab (wrapper)");
        wgg.grabWrapper();
        Thread.sleep(1000);
        runner.log("Release (wrapper)");
        wgg.releaseWrapper();
        Thread.sleep(1000);
        runner.log("Done");
        return true;
    }
}
