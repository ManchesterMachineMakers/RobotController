package org.firstinspires.ftc.teamcode;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.drivebase.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;
import org.firstinspires.ftc.teamcode.subassemblies.Delivery;
import org.firstinspires.ftc.teamcode.diagnostics.util.DiagnosticsOpMode;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp
public class Diagnostics extends DiagnosticsOpMode {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Testable[] provides() {
        return RobotHardware.CURRENT.getTestable(hardwareMap);
    }

}
