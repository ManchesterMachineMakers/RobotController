// IMPORTANT: I pulled this from https://github.com/ErikOverflow/FtcRobotController/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/FtcDashboard_Camera.java
package org.firstinspires.ftc.teamcode

import android.graphics.Bitmap
import android.graphics.Canvas
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.subassemblies.Arm
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.atomic.AtomicReference

@Autonomous(name = "Dash Camera", group = "Linear")
class DashCamera : LinearOpMode() {
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

        override fun onDrawFrame(canvas: Canvas, i: Int, i1: Int, v: Float, v1: Float, o: Any) {}
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val mecDriveBase = MecDriveBase(this)
        val arm = Arm(this)
        val processor = CameraStreamProcessor()
        VisionPortal.Builder().addProcessor(processor)
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()
        waitForStart()
        FtcDashboard.getInstance().startCameraStream(processor, 0.0)
        while (opModeIsActive()) {
            mecDriveBase.control(gamepad1)
            arm.control(gamepad2)
            sleep(100)
        }
    }
}