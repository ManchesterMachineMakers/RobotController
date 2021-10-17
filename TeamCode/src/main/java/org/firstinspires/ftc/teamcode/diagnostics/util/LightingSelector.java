package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;

public class LightingSelector implements Selector<Blinkin> {
    public LightingSelector(Blinkin ledUtil) {
        this.select(ledUtil);
    }
    private Blinkin selection;
    @Override
    public void select(Blinkin inst) {
        selection = inst;
    }

    @Override
    public Blinkin get() {
        return selection;
    }
}
