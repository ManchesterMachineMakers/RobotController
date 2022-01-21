package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drivebase.*;



//***************************************************************************************************************************
@TeleOp(name = "New TeleArray")

public class TeleArrayNew extends LinearOpMode {
  private DriveBase driveBase;


//*************************************************************************************************************************
  public void runOpMode() {
    driveBase = new InchwormMecanumDriveBase(this);
    driveBase.setTravelDirection(DriveBase.TravelDirection.forward);
    double r;
    double robotAngle;
    double rightX;
    double v1;
    double v2;
    double v3;
    double v4;
//------------------------------------------------------------------------------------
    // Prompt user to push start button.
    telemetry.addData("New TeleArray initialized, ", "Press start to continue...");
    telemetry.update();
    // Wait until user pushes start button.
    waitForStart();
    while(opModeIsActive()) {
      r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
      robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
      rightX = gamepad1.right_stick_x;
      v1 = r * Math.cos(robotAngle) + rightX;
      v2 = r * Math.sin(robotAngle) - rightX;
      v3 = r * Math.sin(robotAngle) + rightX;
      v4 = r * Math.cos(robotAngle) - rightX;
      telemetry.addData("V1", v1);
      telemetry.addData("V2", v2);
      telemetry.addData("V3", v3);
      telemetry.addData("V4", v4);
      telemetry.update();
      driveBase.go(new double[]{v1, v2, v3, v4});
    }
  }
}
