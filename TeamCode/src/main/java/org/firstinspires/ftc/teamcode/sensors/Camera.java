package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.teamcode.navigation.org.firstinspires.ftc.teamcode.util.pathfinder.Destination;
//import org.firstinspires.ftc.teamcode.navigation.FieldDestinations;
import org.firstinspires.ftc.teamcode.util.RecognitionMatrix;
import org.firstinspires.ftc.teamcode.util.RobotConfig;

import java.util.List;

@Deprecated // Ultimate Goal
public class Camera {

    public WebcamName webCam1;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;

    private static final String VUFORIA_KEY = //"@string/vuforia_license_key";
            RobotConfig.CURRENT.getValue("vuforiaKey");

    private TensorFlowObjectDetector tfod;

    //public VuforiaNavCamera vuforia;
    private LinearOpMode opMode;
    private VuforiaCurrentGame vuforiaCurrentGame;

    public Camera(HardwareMap hardwareMap, LinearOpMode opMode) {
        /**
         * Copied from Copied from sample code - this went at the beginning of the OpMode to initialize the camera views.
         **/

        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        initVision(cameraMonitorViewId, tfodMonitorViewId, opMode);
    }
    // keep this instance safe; we don't want to update it once we have it.
    private RecognitionMatrix detectedDucks;
    public RecognitionMatrix getDucks() {
        if (detectedDucks == null) {
            detectedDucks = detectDucksUsingTensorFlow();
        }
        return detectedDucks;
    }

    //public VuforiaNavCamera vuforiaForTfod = new VuforiaNavCamera();
    //public TensorFlowObjectDetector tensorflow = new TensorFlowObjectDetector();
    public VuforiaTrackables vuMarkTargets = null;

    public Telemetry telemetry = null;

    //public int cameraMonitorViewId = 0;
    //public int tfodMonitorViewId = 0;

    public boolean isItADuck(RecognitionMatrix duck) {
        if (duck.recognition.getConfidence() > 0.7) {
            // it's probably a duck.
            return true;
        }
        return false;
    }


    public RecognitionMatrix detectDucksUsingTensorFlow() {
        // implement detectDucks using TensorFlowLite
        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions == null) {
            RobotLog.i("Recognition list is null.");
            return null; }

        RobotLog.i("Recognitions size: " + recognitions.size());
        tfod.printRecognitions(recognitions, telemetry);
        List<Recognition> recognitionList = tfod.recognitionsByLabel(recognitions, "duck");
        // recognitionList.addAll(tfod.recognitionsByLabel(recognitions, "Single"));
        if(recognitionList.size() > 0) {
            for (Recognition duck: recognitionList
                 ) {
                RobotLog.i("*16221 Camera*", "Recognized a duck: [" + duck.getLabel() + "] (" + duck.getConfidence() + ")");
                RecognitionMatrix matrix = new RecognitionMatrix(duck);
                if (isItADuck(matrix)) {
                    return matrix;
                }
            }
        } 
        return null;
    }

    public OpenGLMatrix detectWobbleGoalUsingTensorFlow() {
        // TODO: implement wobble goal detection using TensorFlowLite (custom model)
        return null;
    }

//    public OpenGLMatrix getRobotLocationUsingVuMarks() {
//        return vuforia.getRobotLocationFromVuMarks();
//    }

    public VectorF getRobotTranslationUsingVuMarks() {
        return null; //Navigation.getTranslationVector(vuforia.getRobotLocationFromVuMarks(), telemetry);
    }

    public void initVision(int cameraMonitorViewId, int tfodMonitorViewId, LinearOpMode opMode) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first, then the object detector.

        // create Vuforia object
        vuforiaCurrentGame = new VuforiaCurrentGame();

        VuforiaLocalizer vl1;
        vuforiaCurrentGame.initialize(
            VUFORIA_KEY, //java.lang.String vuforiaLicenseKey,
            CAMERA_CHOICE, //VuforiaLocalizer.CameraDirection cameraDirection,
            true, //boolean useExtendedTracking,
            true, //boolean enableCameraMonitoring,
            VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, //VuforiaLocalizer.Parameters.CameraMonitorFeedback cameraMonitorFeedback,
            0, //float dx,
            0, //float dy,
            0, //float dz,
            AxesOrder.XYZ, //AxesOrder axesOrder,
            0, //float firstAngle,
            0, //float secondAngle,
            0, //float thirdAngle,
            true //boolean useCompetitionFieldTargetLocations
        );

        // The other camera is for navigation using vumarks.  We will load all trackables to it.
        vl1 = vuforiaCurrentGame.getVuforiaLocalizer(); // vuforia.initVuforia(cameraMonitorViewId, webCam1, telemetry);
        //vuMarkTargets = vuforia.initTrackables(vl1);

        tfod = new TensorFlowObjectDetector(tfodMonitorViewId, vl1);
    }

    public void cleanupVision() {
        vuforiaCurrentGame.deactivate();
        //vuforia.cleanupVuforia();
        tfod.cleanupTfod();
        vuforiaCurrentGame = null;
        vuMarkTargets = null;
        this.telemetry.addLine("Cleaning up vision functionality.");
    }


}
