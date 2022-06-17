package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.rutins.aleks.diagonal.Subject;

import org.firstinspires.ftc.teamcode.util.Subassembly;

public class Gamepad implements Subassembly, Subject {
    private final OpMode opMode;

    public Gamepad(OpMode om) {
        this.opMode = om;
    }

    public com.qualcomm.robotcore.hardware.Gamepad get(int num) throws Exception {
        return (com.qualcomm.robotcore.hardware.Gamepad) (OpMode.class.getField("gamepad" + num)).get(this.opMode);
    }
}
