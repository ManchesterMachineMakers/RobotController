package org.firstinspires.ftc.teamcode.util

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.atomic.AtomicReference

interface DashOpMode {

    //Talk Volatile vs AtomicReference, final to make sure it is only initialized once, etc.
    class CameraStreamProcessor : VisionProcessor, CameraStreamSource {
        private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

        //private volatile Bitmap lastFrame =  Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565);
        override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>?>) {
            continuation.dispatch { bitmapConsumer: Consumer<Bitmap>? ->
                bitmapConsumer!!.accept(
                    lastFrame.get()
                )
            }
        }

        override fun init(width: Int, height: Int, cameraCalibration: CameraCalibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565))
        }

        override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
            val b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
            Utils.matToBitmap(frame, b)
            lastFrame.set(b)
            return null
        }

        override fun onDrawFrame(canvas: Canvas, i: Int, i1: Int, v: Float, v1: Float, o: Any?) {
        }
    }

    object Static {
        @JvmStatic val dashboard = FtcDashboard.getInstance()
        @JvmStatic val telemetry = dashboard.telemetry
    }

    fun joystickTelemetry(opMode: OpMode) {
        val telemetry = opMode.telemetry
        val g1 = opMode.gamepad1
        val g2 = opMode.gamepad2
        telemetry.addLine("Joystick Values")
        telemetry.addData("G1 Left X", g1.left_stick_x)
        telemetry.addData("G1 Left Y", g1.left_stick_y)
        telemetry.addData("G1 Right X", g1.right_stick_x)
        telemetry.addData("G1 Right Y", g1.right_stick_x)
        telemetry.addData("G2 Left X", g2.left_stick_x)
        telemetry.addData("G2 Left Y", g2.left_stick_y)
        telemetry.addData("G2 Right X", g2.right_stick_x)
        telemetry.addData("G2 Right Y", g2.right_stick_y)
    }
}