//package org.firstinspires.ftc.teamcode.diagnostics.tests
//
//import org.firstinspires.ftc.teamcode.diagnostics.tests.Base
//import org.firstinspires.ftc.teamcode.sensors.FourCorners.Distances
//import org.firstinspires.ftc.robotcore.external.Telemetry
//import org.firstinspires.ftc.teamcode.sensors.FourCorners.DistanceListener
//import org.firstinspires.ftc.teamcode.sensors.FourCorners
//import androidx.annotation.RequiresApi
//import android.os.Build
//import org.firstinspires.ftc.teamcode.diagnostics.Runner
//import org.firstinspires.ftc.teamcode.diagnostics.util.Testable
//import java.util.ArrayList
//
//@Test("Four Corners")
//class FourCornersTest : Base {
//    var measured = ArrayList<Distances>()
//    private fun startThreadedDetector(runner: Runner, listener: DistanceListener) {
//        // measure distances
//        FourCorners.clearListeners()
//        FourCorners.addListener(listener)
//        FourCorners.startThread(runner.opMode.hardwareMap, runner.opMode)
//    }
//
//    @RequiresApi(api = Build.VERSION_CODES.N)
//    override fun run(sel: Array<Testable>, runner: Runner): Boolean {
//        // Place the robot in a known location on the field, verify coordinates.
//        val telemetry = runner.opMode.telemetry
//        val distanceLine = telemetry.addLine("Measured Distances")
//        val xStatus = distanceLine.addData("Detected X", false)
//        val xItem = distanceLine.addData("X Value", 0)
//        val yStatus = distanceLine.addData("Detected Y", false)
//        val yItem = distanceLine.addData("Y Value", 0)
//        startThreadedDetector(runner) { distances: Distances ->
//            measured.add(distances)
//            xStatus.setValue(distances.statusIRx)
//            xItem.setValue(distances.x)
//            yStatus.setValue(distances.statusIRy)
//            yItem.setValue(distances.y)
//            telemetry.update()
//        }
//        runner.opMode.sleep(3000)
//        // now report our findings in the log.
//        var sumX = 0.0
//        var sumY = 0.0
//        var minX = Double.MAX_VALUE
//        var minY = Double.MAX_VALUE
//        var maxX = Double.MIN_VALUE
//        var maxY = Double.MIN_VALUE
//        val avgX: Double
//        val avgY: Double
//        var foundX = 0
//        var foundY = 0
//        for (`val` in measured) {
//            if (`val`.statusIRx) {
//                sumX += `val`.x
//                foundX++
//                minX = java.lang.Double.min(`val`.x, minX)
//                maxX = java.lang.Double.max(`val`.x, maxX)
//            }
//            if (`val`.statusIRy) {
//                sumY += `val`.y
//                foundY++
//                minY = java.lang.Double.min(`val`.y, minY)
//                maxY = java.lang.Double.max(`val`.y, maxY)
//            }
//        }
//
//        // get the average measured values
//        avgX = sumX / foundX
//        avgY = sumY / foundY
//
//        // report average, min, max, inconsistencies in the ability of the X and Y measurements to be obtained
//        runner.log("Four Corners Results (X):"
//                + " Min X: " + minX
//                + " Max X: " + maxX
//                + " Average X: " + avgX
//                + " Detected X " + foundX + " times of " + measured.size + " measurements."
//        )
//        runner.log("Four Corners Results (Y): "
//                + " Min Y: " + minY
//                + " Max Y: " + maxY
//                + " Average Y: " + avgY
//                + " Detected Y " + foundY + " times of " + measured.size + " measurements."
//        )
//        return false
//    }
//}