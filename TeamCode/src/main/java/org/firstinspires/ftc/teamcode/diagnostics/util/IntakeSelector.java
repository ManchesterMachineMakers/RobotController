package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.subassemblies.ActiveIntake;

public class IntakeSelector implements Selector<ActiveIntake> {
    public IntakeSelector(ActiveIntake activeIntake) {
        this.select(activeIntake);
    }
    private ActiveIntake selection;

    @Override
    public Class<ActiveIntake> type() {
        return ActiveIntake.class;
    }

    @Override
    public void select(ActiveIntake inst) {
        selection = inst;
    }

    @Override
    public ActiveIntake get() {
        return selection;
    }
}
