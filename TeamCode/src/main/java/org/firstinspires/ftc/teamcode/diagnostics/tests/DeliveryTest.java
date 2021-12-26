package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selector;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

@Test("Delivery Test")
public class DeliveryTest implements Base {

    public boolean run(Selector[] sel, Runner runner) throws Exception {
        Delivery delivery = Selector.getOrDie(sel, Delivery.class).get();
        runner.log("Down");
        delivery.down();
        Thread.sleep(1000);
        runner.log("Grab");
        delivery.grab();
        Thread.sleep(1000);
        runner.log("Up");
        delivery.up();
        Thread.sleep(1000);
        runner.log("Release");
        delivery.release();
        Thread.sleep(1000);
        runner.log("Deliver to High Level");
        delivery.deliverHigh();
        Thread.sleep(1000);
        runner.log("Deliver to Mid Level");
        delivery.deliverMid();
        Thread.sleep(1000);
        runner.log("Deliver to Low Level");
        delivery.deliverLow();
        Thread.sleep(1000);
        runner.log("Rest");
        delivery.rest();
        Thread.sleep(1000);
        runner.log("Done");
        return true;
    }
}
