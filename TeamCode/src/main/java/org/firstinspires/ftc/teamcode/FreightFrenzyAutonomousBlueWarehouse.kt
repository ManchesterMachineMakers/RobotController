package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.pathfinder.FieldDestinations2021

@Autonomous(name = "Freight Frenzy Autonomous (Blue Alliance by Warehouse)", group = "Freight Frenzy")
class FreightFrenzyAutonomousBlueWarehouse : FreightFrenzyAutonomous(Alliance.Blue, FieldDestinations2021.BlueStart1)