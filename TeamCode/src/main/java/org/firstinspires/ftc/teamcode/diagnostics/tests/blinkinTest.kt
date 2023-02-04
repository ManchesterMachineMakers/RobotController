package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.Blinkin

fun blinkinTest(opMode: LinearOpMode) = describe<Blinkin> { blinkin ->
    runner.log("Autonomous default")
    blinkin.autonomousDefault()
    opMode.sleep(1000)

    runner.log("Autonomous action")
    blinkin.autonomousAction()
    opMode.sleep(1000)

    runner.log("Autonomous action alert")
    blinkin.autonomousActionAlert()
    opMode.sleep(1000)

    runner.log("Detecting")
    blinkin.detecting()
    opMode.sleep(1000)

    runner.log("Teleop default")
    blinkin.teleOpDefault()
    opMode.sleep(1000)

    runner.log("Endgame default")
    blinkin.endgameDefault()
    opMode.sleep(1000)

    runner.log("Game over")
    blinkin.gameOver()
    runner.log("Alert, then reset")
    blinkin.alertThenReset()
}