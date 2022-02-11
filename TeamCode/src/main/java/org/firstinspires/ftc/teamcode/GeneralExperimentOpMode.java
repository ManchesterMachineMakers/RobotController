package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.SoundEffects;

@TeleOp(name = "Experimental", group = "Diagnostics")
public class GeneralExperimentOpMode extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SoundEffects se;
        try {
            se = RobotConfig.CURRENT.getHardware(SoundEffects.class, this);
            waitForStart();
            if(opModeIsActive()) {
                se.play("silver");
            }
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }
    }
}