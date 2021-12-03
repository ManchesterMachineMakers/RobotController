package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.sensors.RingSensor;

/**
 * This is for a threaded detector - the test will instantiate the
 * class in a new thread.
 */
public class RingSensorSelector implements Selector<RingSensor> {

    public RingSensorSelector() {
        this.select(null);
    }

    @Override
    public void select(RingSensor inst) {
        ;
    }

    @Override
    public RingSensor get() {
        return null;
    }
}
