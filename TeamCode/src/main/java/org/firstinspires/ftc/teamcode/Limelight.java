package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Limelight {

    Limelight3A limelight;

    boolean hasFoundSample = false;

    ElapsedTime timer;


    Double[] previousSampleLockAngles = new Double[5];

    RobotMain bart;

    double SAMPLE_HEIGHT = 1.5;
    double MOUNTING_ANGLE_RADIANS = Math.toRadians(-65);
    double CAMERA_HEIGHT = 7;

    double tx;
    double ty;
    double ta;

    boolean resultExists;

    double distFromCameraToSample;

    double slideTarget;

    public int limelightPipeline;

    LLResult result;


    Limelight(HardwareMap hardwareMap, int pipelineNumber) {
        limelightPipeline = pipelineNumber;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!


        limelight.pipelineSwitch(limelightPipeline);

        tx = 0;
        ty = 0;
        ta = 0;
    }

    public boolean isSeeingResult() {
        return resultExists;
    }

    public void updateLastLimelightResult() {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa();// How big the target looks (0%-100% of the image)
            resultExists = true;
        } else {
            resultExists = false;
        }
    }

    public double returnLastResultTY() {
        return ty;
    }

    public void setPipeline(int pipeline) {
        limelightPipeline = pipeline;
        limelight.pipelineSwitch(limelightPipeline);
    }

    public double getLastResultDistanceInches() {
        //Estimate Distance
        distFromCameraToSample = (SAMPLE_HEIGHT - CAMERA_HEIGHT) / (Math.tan(MOUNTING_ANGLE_RADIANS + Math.toRadians(tx)));
        distFromCameraToSample = RobotMath.maxAndMin(distFromCameraToSample, 10, 0);
        return distFromCameraToSample;
    }
}
