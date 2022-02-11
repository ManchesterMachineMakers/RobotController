package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;

public class SoundEffects implements Subassembly {

    private OpMode opMode;

    public SoundEffects(OpMode opMode) {
        this.opMode = opMode;
    }

    public void play(String soundName) {
        AndroidSoundPool androidSoundPool = new AndroidSoundPool();

        // Put initialization blocks here.
        SoundPlayer.getInstance().startPlaying(opMode.hardwareMap.appContext, opMode.hardwareMap.appContext.getResources().getIdentifier(soundName, "raw", opMode.hardwareMap.appContext.getPackageName()));

        opMode.telemetry.update();
        androidSoundPool.close();
    }
}
