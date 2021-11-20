package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;

public class DriveBaseSelector implements Selector<DriveBase> {
    public DriveBaseSelector(DriveBase db) {
        this.select(db);
    }
    private DriveBase selection;
    @Override
    public void select(DriveBase inst) {
        selection = inst;
    }

    @Override
    public DriveBase get() {
        return selection;
    }

    @Override
    public Class<DriveBase> type() {
        return DriveBase.class;
    }
}
