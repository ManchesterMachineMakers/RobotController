package org.firstinspires.ftc.teamcode;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

@TeleOp(name = "Full Diagnostic", group = "Diagnostics")
public class Diagnostics extends DiagnosticsOpMode {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Testable[] provides() {
        return RobotConfig.CURRENT.getTestable(this);
    }

}
