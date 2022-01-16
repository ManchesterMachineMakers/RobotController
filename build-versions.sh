#!/usr/bin/env bash
source shell/buildmatrix.sh

matrix/config TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/RobotHardware.java

matrix/build FULL_ROBOT
matrix/build PROGRAMMING_BOARD