package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class Limelight {

    Limelight3A limelight;

    boolean hasFoundSample = false;

    ElapsedTime timer;


    Double[] previousSampleLockAngles = new Double[5];

    RobotMain bart;

    double SAMPLE_HEIGHT = 1.5;
    double MOUNTING_ANGLE_RADIANS = Math.toRadians(-65);
    double CAMERA_HEIGHT = 7;

    /*double tx;
    double ty;
    double ta;*/

    String className;

    boolean resultExists;

    double distFromCameraToSample;

    double slideTarget;

    List<LLResultTypes.DetectorResult> detections = new ArrayList<>();

    public int limelightPipeline;
    //0 yellow
    //1 blue
    //2 red
    //7 yellow
    //8 blue
    //9 red

    LLResult result;
    double tx = 0;
    double ty = 0;
    double distance = 0;


    Limelight(HardwareMap hardwareMap, int pipelineNumber) {
        limelightPipeline = pipelineNumber;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!


        limelight.pipelineSwitch(limelightPipeline);
    }

    public boolean isSeeingResult() {
        return resultExists;
    }

    public void updateLastLimelightResult(double desiredDistance) {
        result = limelight.getLatestResult();
        if (isUsingColor()) {
            updateForColor(desiredDistance);
        } else {
            updateForAI(desiredDistance);
        }
        distance = distanceToTx(tx);
    }

    public void updateForColor(double desiredDistance) {
        if ( result != null && result.isValid() ) {
            tx = result.getTx();
            ty = result.getTy();
            resultExists = true;
        } else {
            resultExists = false;
        }
    }

    public void updateForAI(double desiredDistance) {
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            resultExists = true;
            detections = result.getDetectorResults();
            LLResultTypes.DetectorResult bestDetection = detections.get(0);
            double bestDetectionDistOff = Math.abs(distanceToTx(bestDetection.getTargetXDegrees()) - desiredDistance);
            for (LLResultTypes.DetectorResult detection : detections) {
                double currentDetectionDistOff = Math.abs(distanceToTx(detection.getTargetXDegrees()) - desiredDistance);
                if (currentDetectionDistOff < bestDetectionDistOff) {
                    bestDetection = detection;
                }
            }

            tx = bestDetection.getTargetXDegrees();
            ty = bestDetection.getTargetYDegrees();

        } else {
            resultExists = false;
        }
    }

    public String telemetryOfAll() {
        StringBuilder stringBuilder = new StringBuilder();
        if (!detections.isEmpty()) {
            for (LLResultTypes.DetectorResult detection : detections) {
                stringBuilder.append(telemetryOfDetection(detection));
            }
        }
        return stringBuilder.toString();
    }

    public String telemetryLast() {
        return "\ntx: " + tx +
                "\ndist: " + distance +
                "\nty: " + ty;

    }

    public String telemetryOfDetection(LLResultTypes.DetectorResult detectorResult) {
        return "\ntx: " + detectorResult.getTargetXDegrees() +
                "\ndist: " + distanceToTx(detectorResult.getTargetXDegrees()) +
                "\nty: " + detectorResult.getTargetYDegrees();
    }

    public List<LLResultTypes.DetectorResult> getDetections() {
        return detections;
    }
    public double returnLastResultTY() {
        return ty;
    }

    /*public String returnLastClassName() {
        return lastDetection.getClassName();
    }*/

    public void setPipeline(int pipeline) {
        limelightPipeline = pipeline;
        limelight.pipelineSwitch(limelightPipeline);
    }

    public int getPipeline() {
        return limelightPipeline;
    }

    public boolean isUsingColor() {
        return getPipeline() <= 3;
    }

    /*public double distanceToDetection(LLResultTypes.DetectorResult detection) {
        return distanceToTx(detection.getTargetXDegrees());
    }*/

    public double distanceToTx(double tx) {
        //Estimate Distance
        double distFromCameraToSample = (SAMPLE_HEIGHT - CAMERA_HEIGHT) / (Math.tan(MOUNTING_ANGLE_RADIANS + Math.toRadians(tx)));
        distFromCameraToSample = RobotMath.maxAndMin(distFromCameraToSample, 10, 0);
        return distFromCameraToSample;
    }

    public double getLastResultDistanceInches() {
        return distance;
    }
}
