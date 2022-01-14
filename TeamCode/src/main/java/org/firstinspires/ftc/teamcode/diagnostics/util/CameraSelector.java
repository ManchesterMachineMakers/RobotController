package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.sensors.Vision;

public class CameraSelector implements Selector<Vision> {
    Vision camera;
    public CameraSelector(Vision camera) {
        select(camera);
    }

    @Override
    public <S extends Vision> void select(S inst) {
        this.camera = inst;
    }

    @Override
    public <S extends Vision> S get() {
        return (S) this.camera;
    }
}
