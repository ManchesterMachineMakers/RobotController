#!/usr/bin/env bash
function build() {
    echo "Build debug $1"
    ./gradlew assembleDebug
    mv TeamCode/build/outputs/apk/debug/TeamCode-debug.apk TeamCode-debug-$1.apk
    echo "Build release $1"
    ./gradlew assembleRelease
    mv TeamCode/build/outputs/apk/debug/TeamCode-release.apk TeamCode-release-$1.apk
}
# Mecanum drive base
build mecanum
# Programming board
sed s/MecanumDriveBase/ProgrammingBoardDriveBase/ {TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Diagnostics.java,TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/MMMFreightFrenzyOpMode.java,TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/MMMUltimateGoalOpMode.java}
build programming-board
# Inchworm mecanum
sed s/ProgrammingBoardDriveBase/InchwormMecanumDriveBase/ {TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Diagnostics.java,TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/MMMFreightFrenzyOpMode.java,TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/MMMUltimateGoalOpMode.java}
build inchworm