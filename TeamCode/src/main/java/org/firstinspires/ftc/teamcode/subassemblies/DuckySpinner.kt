package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.KtHardware
import org.firstinspires.ftc.teamcode.util.Subassembly

class DuckySpinner(private val opMode: LinearOpMode) : Subassembly {

    /*
    SPECS: GoBilda Dual Mode Speed Servo
    Weight	58g
    Gear Ratio	135:1
    Output Shaft Style	25 Tooth Spline
    Voltage Range	4.8V ~ 7.4V
    No-Load Speed (4.8V)	0.11 sec/60° (90RPM)
    No-Load Speed (6.0V)	0.09 sec/60° (115RPM)
    No-Load Speed (7.4V)	0.07 sec/60° (145RPM)
    Stall Torque (4.8V)	7.9 kg.cm (110 oz-in)
    Stall Torque (6.0V)	9.3 kg.cm (130 oz-in)
    Stall Torque (7.4V)	10.8 kg.cm (150 oz-in)
    No-Load Current (4.8V)	190mA
    No-Load Current (6.0V)	200mA
    No-Load Current (7.4V)	230mA
    Stall Current (4.8V)	2000mA
    Stall Current (6.0V)	2500mA
    Stall Current (7.4V)	3000mA
    Max PWM Range (Default)	500 - 2500μsec
    Max PWM Range (Continuous)	1000 - 2000μsec
    Travel per µsec (Default)	0.15°/μsec
    Max Travel (Default)	300°
    Pulse Amplitude	3-5V
    Direction	Clockwise w/ Increasing PWM Signal
    Deadband Width	4μsec
    Motor Type	Brushed DC
    Feedback Style	5KΩ Potentiometer
    Output Shaft Support	Dual Ball Bearing
    Gear Material	Steel
    Wire Length	300mm
    Wire Gauge	22 AWG
    Connector Type	3-Pos TJC8 Servo Connector [MH-FC]

    carousel diameter: 38cm
    carousel circumference: 119.380
    wheel diameter: 5cm
    wheel circumference: 15.708
    1 rotation of the carousel = 7.6 rotations of the wheel
    max desired carousel RPM: 42.857
    max desired servo RPM: 325.713
     */


    val duckySpinnerSpeed = 1.0 // max voltage should give us 145 RPM

    val wheel = opMode.hardwareMap.crservo[KtHardware.name("servo_DuckWheel")]
    var running = false
    fun start(power: Double) {
        wheel.power = power
        running = true
    }
    fun stop() {
        wheel.power = 0.0
        running = false
    }
    fun controller() {
        val gamepad = when(KtHardware.name("duckWheel_Gamepad")) {
            "gamepad1" -> opMode.gamepad1
            else -> opMode.gamepad2
        }
        if(gamepad.right_trigger > 0.1) {
            if(!running) start(duckySpinnerSpeed)
        } else if(gamepad.left_trigger > 0.1) {
            if(!running) start(-(duckySpinnerSpeed))
        } else {
            stop()
        }
    }
}