package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.CommendGroups.Camera.CameraCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.MMSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMathTools;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.ArrayList;
import java.util.List;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog

public class Camera extends MMSubsystem {
    public final Limelight3A camera;

    private LLResult detectorPreviousResult;

    public int currentPipeline = 0;

    private boolean initiated = false;


    //detection parameters for distance and strafe:
    public static double CAMERA_HEIGHT = 445;
    public static double CAMERA_ANGLE = 90 - 35.0;
    public static double TARGET_HEIGHT = 39;
    public static double SPECIMEN_HEIGHT = 247.5;


    //size and location of detected sample (-1 means not initialized):
    public static List<Double> targetLeftUp;
    public static double length = -1;
    public static double height = -1;
    public static double x = -1;
    public static double y = -1;

    public static double timesAngleFailed = 0;
    public static double timesPipelineSwitchFail = 0;

    private int sampleColorID; //current color need to be detected
    private static Camera instance;


    public Camera() {
        super();
        MMRobot.getInstance().subsystems.add(this);

        camera = MMRobot.getInstance().currentOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        InitializeCamera();
        camera.pipelineSwitch(currentPipeline);

        targetLeftUp = new ArrayList<>();
        targetLeftUp.add(0, 0.0);
        targetLeftUp.add(0, 0.0);
    }

    public static synchronized Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

    public void InitializeCamera() {
        camera.setPollRateHz(100);
        camera.start();
    }

    public void SetInitiated(){
        initiated = true;
    }

    //Get degrees in X axis getting cameraResult
    public double getTx(LLResult cameraResult, double defaultValue) {
        if (cameraResult == null) {
            return defaultValue;
        }
        return cameraResult.getTx();
    }

    //Get degrees in X axis getting cameraResult
    public double getTy(LLResult cameraResult, double defaultValue) {
        if (cameraResult == null) {
            return defaultValue;
        }
        return cameraResult.getTy();
    }

    //Get distance in Y axis with given cameraResult
    public Double getDistance(LLResult lastResult,double defaultValueTy) {
        double ty = getTy(lastResult, defaultValueTy);
        if (ty == 0) {
            return 0.0;
        }
        double angleToGoalDegrees = CAMERA_ANGLE - ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM);
    }

    public double getStrafeOffset(LLResult lastResult,double defaultValueTy, double defaultValueTx) {
        if (lastResult != null) {
            double tx = getTx(lastResult, defaultValueTx);
            if (tx != defaultValueTx) {
                double tanTX = Math.tan(Math.toRadians(tx));
                double height = CAMERA_HEIGHT - TARGET_HEIGHT;
                double distanceY = getDistance(lastResult, defaultValueTy);
                double diagonalLength = Math.sqrt(height * height + distanceY * distanceY);
                return tanTX * diagonalLength / 2.54 / 10;
            }
        }
        return 0;
    }

    //Get angle of a sample in servo degrees
    public Double getSampleAngle() {
        LLResult cameraResult = camera.getLatestResult();

        if (cameraResult == null) {
            timesAngleFailed += 1;
            return null;
        }
        return cameraResult.getPythonOutput()[0];
    }


    public double getPipelineIndex() {
        return camera.getStatus().getPipelineIndex();
    }

    @Override
    public void resetHub() {}

    public boolean isDataOld(){
        return camera.getLatestResult().getStaleness() >= 100;
    }

    public void setPreviousResult() {
        detectorPreviousResult = camera.getLatestResult();
    }

    //find the closest sample to the middle of the robot
    public void findClosestSample() {
        if (detectorPreviousResult != null) {
            List<LLResultTypes.DetectorResult> detectorResults = detectorPreviousResult.getDetectorResults();
            if (!detectorResults.isEmpty()) {
                LLResultTypes.DetectorResult dr = detectorResults.get(0);
                List<List<Double>> corners = dr.getTargetCorners();
                List<Double> leftUp = corners.get(0);
                List<Double> rightUp = corners.get(1);
                List<Double> rightDown = corners.get(2);
                length = MMathTools.distance(leftUp, rightUp);
                height = MMathTools.distance(rightDown, rightUp);
                targetLeftUp = leftUp;
                x = targetLeftUp.get(0);
                y = targetLeftUp.get(1);
                sampleColorID = dr.getClassId();
            }
        }
    }

    public void trackRed() {
        currentPipeline = 0;
    }

    public void trackBlue() {
        currentPipeline = 1;
    }

    public void trackRedAndYellow() {
        currentPipeline = 6;
    }

    public void trackBlueAndYellow() {
        currentPipeline = 7;
    }

    //Switch to neural-detector based detection pipepline (AI omg ooga booga big words I love man)
    public boolean switchToDetector() {
        if (!camera.pipelineSwitch(currentPipeline)) {
            //telemetry.addData("failed to switch to detector", 0);
            timesPipelineSwitchFail += 1;
            return false;
        }

        return true;
    }

    //Switch to python based detection pipepline
    public boolean switchToPython() { //TODO : Check numbers if true
        if (sampleColorID == 0){
            currentPipeline = 5; // Blue
        }
        else if (sampleColorID == 1){
            currentPipeline = 3; // Red
        }
        else if (sampleColorID == 2){
            currentPipeline = 4; // Yellow
        }

        if (!camera.pipelineSwitch(currentPipeline)) {
            //telemetry.addData("failed to switch to python", sampleColorID);
            timesPipelineSwitchFail += 1;
            return false;
        }
        return true;
    }

    public LLResult GetResult(){
        return camera.getLatestResult();
    }

    public LLResult GetPreviousDetectorResult(){
        return detectorPreviousResult;
    }

    //Only change the angle of the intake rotator
    public SequentialCommandGroup changeRotatorAngle() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> findClosestSample()),
                new WaitUntilCommand(() -> switchToPython()),

                new WaitUntilCommand(() -> camera.getStatus().getPipelineIndex() == currentPipeline),
                new InstantCommand(() -> camera.updatePythonInputs(new double[]{0.0, 0.0, 0.0, length, height, x, y, 0.0})),
                new WaitUntilCommand(() -> camera.getLatestResult().getPythonOutput()[0] != 0),
                CameraCommands.RotateToSampleCommand()
        );
    }

    //Doing every moment, it updates the python inputs, and then updates the cameraResult to the latest and freshest one. and telemtry, a lot of telemtry.
    @Override
    public void periodic() {
        //updating the python endlessly
        if (!initiated) return;

        camera.updatePythonInputs(
                new double[]{0.0, 0.0, 0.0, length, height, x, y, 0.0}
        );
    }
}