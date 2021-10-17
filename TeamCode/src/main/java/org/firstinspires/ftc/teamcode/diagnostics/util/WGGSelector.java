package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.subassemblies.WobbleGoalGrabber;

public class WGGSelector implements Selector<WobbleGoalGrabber> {
    WobbleGoalGrabber wgg;
    public WGGSelector(WobbleGoalGrabber wgg) {
        select(wgg);
    }
    public <S extends WobbleGoalGrabber> void select(S inst) {
        this.wgg = inst;
    }

    public <S extends WobbleGoalGrabber> S get() {
        return (S) this.wgg;
    }
}
