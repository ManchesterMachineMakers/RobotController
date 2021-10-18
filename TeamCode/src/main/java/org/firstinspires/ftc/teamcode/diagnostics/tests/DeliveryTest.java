package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

@Test("Wobble Goal Grabber Test")
public class DeliveryTest implements Base {

    public boolean run(Selectors sel, Runner runner) throws InterruptedException {
        Delivery delivery = sel.deliverySelector().get();
        runner.log("Up");
        delivery.up();
        Thread.sleep(1000);
        runner.log("Grab");
        delivery.grab();
        Thread.sleep(1000);
        runner.log("Release");
        delivery.release();
        Thread.sleep(1000);
        runner.log("Down");
        delivery.down();
        Thread.sleep(1000);
        runner.log("Up");
        delivery.up();
        Thread.sleep(1000);
        runner.log("Grab (wrapper)");
        delivery.grabWrapper();
        Thread.sleep(1000);
        runner.log("Release (wrapper)");
        delivery.releaseWrapper();
        Thread.sleep(1000);
        runner.log("Done");
        return true;
    }
}
