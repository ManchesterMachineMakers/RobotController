package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.DriveBase.TravelDirection

fun mecanumCoefficientsForDirection(direction: TravelDirection)
    = when (direction) {
        TravelDirection.base, TravelDirection.forward -> arrayOf( 1,  1,  1,  1)
        TravelDirection.reverse ->                       arrayOf(-1, -1, -1, -1)
        TravelDirection.pivotLeft ->                     arrayOf(-1,  1, -1,  1)
        TravelDirection.pivotRight ->                    arrayOf( 1, -1,  1, -1)
        TravelDirection.strafeLeft ->                    arrayOf(-1,  1,  1, -1)
        TravelDirection.strafeLeftForward ->             arrayOf( 0,  1,  1,  0)
        TravelDirection.strafeLeftBackward ->            arrayOf(-1,  0,  0, -1)
        TravelDirection.strafeRight ->                   arrayOf( 1, -1, -1,  1)
        TravelDirection.strafeRightForward ->            arrayOf( 1,  0,  0,  1)
        TravelDirection.strafeRightBackward ->           arrayOf( 0, -1, -1,  0)
        TravelDirection.pitch ->                         arrayOf( 0,  0,  0,  0)
    }