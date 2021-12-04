package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.sensors.Camera;

public class CameraSelector implements Selector<Camera> {
    Camera camera;
    public CameraSelector(Camera camera) {
        select(camera);
    }

    @Override
    public <S extends Camera> void select(S inst) {
        this.camera = inst;
    }

    @Override
    public <S extends Camera> S get() {
        return (S) this.camera;
    }
}
