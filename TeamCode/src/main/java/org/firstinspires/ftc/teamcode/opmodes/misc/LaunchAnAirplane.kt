package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.manchestermachinemakers.easyop.Device
import org.manchestermachinemakers.easyop.Linear

@TeleOp(name = "Launch An Airplane")
class LaunchAnAirplane : Linear() {
    @Device
    lateinit var airplaneLauncher: DcMotor

    override fun opBeforeLoop() {
        airplaneLauncher.power = 0.4
    }
}