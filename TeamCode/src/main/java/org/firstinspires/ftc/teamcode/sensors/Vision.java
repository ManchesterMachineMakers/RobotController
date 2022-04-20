package org.firstinspires.ftc.teamcode.sensors;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.rutins.aleks.diagonal.Subject;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.List;

public class Vision implements Subassembly, Subject {
    public static class TFODNotInitializedException extends Exception {
        public TFODNotInitializedException() {
            super("TensorFlow not initialized.");
        }
    }

    private final HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    private VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    public Vision(LinearOpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
    }

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "ManchesterMachineMuffin.tflite";
    private static final String[] LABELS = {
            "muffin"
    };
    private static final String[] VUMARKS = {
            "Blue Storage",
            "Blue Alliance Wall",
            "Red Storage",
            "Red Alliance Wall"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = RobotConfig.CURRENT.getValue("vuforiaKey");

    /**
     * This is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;
    public VuforiaCurrentGame currentGame;

    /**
     * This is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    private VuforiaLocalizer.Parameters getVuforiaParameters() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;
        parameters.fillCameraMonitorViewParent = true;

        return parameters;

    }
    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforiaForVuMarks() {
        String webcamCalibrationFilename = "";
        boolean enableCameraMonitoring = true;
        float dx = 0;
        float dy = 0;
        float dz = 0;
        float firstAngle = 0;
        float secondAngle = 0;
        float thirdAngle = 0;
        boolean useCompetitionFieldTargetLocations = true;

        // Set up the current VuMarks
        currentGame = new VuforiaCurrentGame();
        VuforiaLocalizer.Parameters parameters = getVuforiaParameters();
        currentGame.initialize(
                parameters.vuforiaLicenseKey,
                parameters.cameraName,
                webcamCalibrationFilename,
                parameters.useExtendedTracking,
                enableCameraMonitoring,
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                dx,
                dy,
                dz,
                AxesOrder.XYZ,
                firstAngle,
                secondAngle,
                thirdAngle,
                useCompetitionFieldTargetLocations
        );
        currentGame.activate();
    }

    public void initVuforiaForTFOD() {

        VuforiaLocalizer.Parameters parameters = getVuforiaParameters();
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    public VuforiaBase.TrackingResults lookForVuMarks(Telemetry telemetry) {
        for (String vuMarkTarget :
                VUMARKS) {
            float x = 0;
            float y = 0;
            telemetry.addData("Target: ", vuMarkTarget)
                    .addData("X: ", x)
                    .addData("Y: ", y);
            VuforiaBase.TrackingResults vuforiaResults = currentGame.track(vuMarkTarget);
            if (vuforiaResults.isVisible) {
                x = vuforiaResults.x;
                y = vuforiaResults.y;
                telemetry.update();
                return vuforiaResults;
            }
        }
        return null;
    }

    public void deactivateTFOD() {
        vuforia.close();
    }

    public void deactivateVuMarks() {
        currentGame.deactivate();
        currentGame.close();
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     **/
    public void activateTfod() {
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }
    }

    /**
     * Update recognitions
     */
    @SuppressLint("DefaultLocale")
    public void spotObjects(Telemetry telemetry) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
                telemetry.update();
            }
        }
    }

    public List<Recognition> getDefiniteRecognitions() throws TFODNotInitializedException {
        if(tfod == null) throw new TFODNotInitializedException();
        long start = System.currentTimeMillis();
        long timeout = RobotConfig.CURRENT.<Integer>getValue("detectionTimeoutMillis").longValue();
        List<Recognition> recognitions = tfod.getRecognitions();

        while(
                (System.currentTimeMillis() - start) < timeout
                && recognitions.size() == 0
                && opMode.opModeIsActive()
        ) {
            recognitions = tfod.getRecognitions();
            opMode.idle();
        }
        return recognitions;
    }
}
