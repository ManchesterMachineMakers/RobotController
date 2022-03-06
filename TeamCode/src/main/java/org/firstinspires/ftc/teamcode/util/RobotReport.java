package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.navigation.Destination;
import org.firstinspires.ftc.teamcode.navigation.FieldDestinations;
import org.firstinspires.ftc.teamcode.navigation.Orientation;
import org.firstinspires.ftc.teamcode.navigation.PIDReckoning;
import org.firstinspires.ftc.teamcode.subassemblies.Blinkin;

public class RobotReport {

    private Telemetry telemetry;
    private Blinkin led;
    private PIDReckoning pidReckoning;

    public Telemetry.Item itemRingsOnBot;
    public Telemetry.Item itemFlywheelSpeed;
    public Telemetry.Item itemIntakeSpeed;
    public Telemetry.Item itemFieldPosition;
    public Telemetry.Item itemFieldOrientation;
    public Telemetry.Item itemInchwormAngle;
    public Telemetry.Item itemShooterAngle;
    public Telemetry.Item itemShooterTarget;
    public Telemetry.Item itemOriginDestination;
    public Telemetry.Item itemTravelDestination;
    public Telemetry.Item itemRunReport;

    // keep these values across opmodes
    public static int ringsOnBot = 0;
    public static Destination fieldPosition = FieldDestinations.SLC;
    public static Orientation fieldOrientation;
    public static float inchwormAngle = 0;
    public static float shooterAngle = 0;

    public Destination originDestination = FieldDestinations.SLC;
    public Destination travelDestination = FieldDestinations.SLC;
    public Destination shooterTarget = FieldDestinations.TOW;
    public String runReport = "Instantiated";

    public double flywheelSpeed = 0;
    public double intakeSpeed=0;
    public boolean isShooterUpToSpeed = false;
    public boolean atShootingPosition = false;
    public boolean atShootingAngle = false;

    /**
     * Create a robot report mechanism.  Supply the telemetry on which to report.
     * Telemetry items may be set individually on the public members of the class,
     * or passed in all at once on an update method.  Items may be reported on
     * the telemetry or into the match log.
     *
     * @param telemetry
     */
    public RobotReport(Telemetry telemetry, Blinkin led) {
        this.led = led;
        this.telemetry = telemetry;
        itemRunReport = telemetry.addLine("RUNNING: ").addData("Stage", "Telemetry Initialization");
        Telemetry.Line intakeLine = telemetry.addLine("Intake");
        itemIntakeSpeed = intakeLine.addData("Intake Speed", (Object)0);
        itemRingsOnBot = intakeLine.addData("Rings on the bot", (Object)0);

        Telemetry.Line shooterLine = telemetry.addLine("Shooter");
        itemFlywheelSpeed = shooterLine.addData("Flywheel Speed", (Object)0);
        itemInchwormAngle = shooterLine.addData("Inchworm Base Angle", (Object)0);
        itemShooterAngle = shooterLine.addData("Shooter Angle", (Object)0);
        itemShooterTarget = shooterLine.addData("Target", FieldDestinations.TOW);

        Telemetry.Line travelLine = telemetry.addLine("Travel");
        itemOriginDestination = travelLine.addData("Origin", FieldDestinations.SLC);
        itemTravelDestination = telemetry.addData("org.firstinspires.ftc.teamcode.util.pathfinder.Destination", new Destination());

        Telemetry.Line positionLine = telemetry.addLine("Current Field Position");
        itemFieldPosition = positionLine.addData("Position", new Destination());
        itemFieldOrientation = positionLine.addData("Orientation", "");

        telemetry.update();
    }

    /**
     * Report current values on the telemetry.
     */
    public void update() {
        telemetry.update();
    }

    /**
     * Update all values and report on the telemetry.
     *
     * @param ringsOnBot
     * @param flywheelSpeed
     * @param isShooterUpToSpeed
     * @param intakeSpeed
     * @param shooterTarget
     * @param fieldPosition
     * @param fieldOrientation
     * @param inchwormAngle
     * @param shooterAngle
     * @param originDestination
     * @param targetDestination
     */
    public void update(int ringsOnBot, float flywheelSpeed, boolean isShooterUpToSpeed, float intakeSpeed, Destination shooterTarget, Destination fieldPosition, Orientation fieldOrientation,
                       float inchwormAngle, float shooterAngle, Destination originDestination, Destination targetDestination) {

        this.intakeSpeed = intakeSpeed;
        this.itemIntakeSpeed.setValue(String.valueOf(intakeSpeed));

        this.fieldPosition = fieldPosition;
        this.itemFieldPosition.setValue(String.valueOf(fieldPosition.getX()) + ", " + String.valueOf(fieldPosition.getY()));

        this.fieldOrientation = fieldOrientation;
        if (fieldOrientation!=null) {
            this.itemFieldOrientation.setValue(String.valueOf(fieldOrientation.psi));
        }

        this.originDestination = originDestination;
        this.itemOriginDestination.setValue(originDestination.getName());

        this.travelDestination = targetDestination;
        this.itemTravelDestination.setValue(targetDestination.getName());

        this.update();
    }

    /**
     * Output current report to the match log.
     */
    public void logReport() {

        String psi = "";
        if (fieldOrientation != null) {
            psi = String.valueOf(fieldOrientation.psi) + ", " + String.valueOf(fieldOrientation.theta);
        }
        RobotLog.ii("*16221 Robot Status Report*",
                "Rings: " + String.valueOf(ringsOnBot) + "\n" +
                        "Flywheel Speed: " + String.valueOf(flywheelSpeed) + "\n" +
                        "Intake Speed: " + String.valueOf(intakeSpeed) + "\n" +
                        "Field Position: " + String.valueOf(fieldPosition.getX()) + ", " + String.valueOf(fieldPosition.getY()) + "\n" +
                        "Field Orientation: " + psi + "\n" +
                        "Inchworm Base Angle: " + String.valueOf(inchwormAngle) + "\n" +
                        "Travel Origin: " + String.valueOf(originDestination.getX()) + ", " + String.valueOf(originDestination.getY()) + "\n" +
                        "Travel Target: " + String.valueOf(travelDestination.getX()) + ", " + String.valueOf(travelDestination.getY()) + "\n"
        );
    }

    public void updateBotStatus() {
/*
        if (ringsOnBot >= ActiveIntake.maxRingsAllowedOnBot) {
            led.ringMagazineFull();
        } else {
            led.needMoreRings();
        }

        // calculate shooting for driver feedback
        MyPath path = Movement.calculatePath(fieldPosition, shooterTarget, fieldOrientation);

        if (path.distance > (Shooter.rangeAtDefaultAngle - Shooter.rangeTolerance)
                && path.distance < (Shooter.rangeAtDefaultAngle + Shooter.rangeTolerance)) {
            atShootingPosition = false;
        } else {
            atShootingPosition = true;
        }
        if (Math.abs(path.leastRot) <= Shooter.headingTolerance
            // && path.pitch <= (this.shooterAngle + Shooter.pitchTolerance)
            // && path.pitch >= (this.shooterAngle - Shooter.pitchTolerance) //TODO: pitch calculations in MyPath.
        ){
            atShootingAngle = true;
        } else {
            atShootingAngle = false;
        }

        // shooter status lights
        if (isShooterUpToSpeed && atShootingPosition && atShootingAngle) {
            led.readyToShoot();
        } else if (isShooterUpToSpeed || atShootingPosition || atShootingAngle){
            led.almostReadyToShoot();
        } else if (!atShootingPosition) {
            led.outOfRange();
        }

 */
    }

    //TODO: More status reports
}
