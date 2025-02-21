package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoPoses {

    static Pose2d beginPoseClips = new Pose2d(-5.5, 63.75, Math.toRadians(270));


    static Pose2d scorePreloadClipsPose = new Pose2d(-3.5, 35, Math.toRadians(270));

    static Pose2d dropSubClipsPose = new Pose2d(-33, 44, Math.toRadians(120));

    static Pose2d grabSpark1ClipsPose = new Pose2d(-31, 40, Math.toRadians(240));//-36,36
    static Pose2d grabSpark2ClipsPose = new Pose2d(-42, 40, Math.toRadians(250));//-41, 40
    static Pose2d grabSpark3ClipsPose = new Pose2d(-45.5, 24, Math.toRadians(200));

    static Pose2d dropSpark1ClipsPose = new Pose2d(grabSpark2ClipsPose.position, Math.toRadians(130));
    static Pose2d dropSpark2ClipsPose = new Pose2d(-44, 40, Math.toRadians(160));

    static Pose2d grabWallClipsPose = new Pose2d(-41, 61, Math.toRadians(90));

    static Pose2d scoreCycleClipsPose = new Pose2d(-5, 30.25, Math.toRadians(90));

    static Pose2d clipsParkPose = shiftPoseByInputs(grabWallClipsPose, -6, -14, 0);



    static double preloadMinAccel = -70;
    static double preloadMaxAccel = 70;
    static double preloadMaxWheelVel = 70;

    static double scoreCycleMinAccel = -75;
    static double scoreCycleMaxAccel = 70;
    static double scoreCycleGrabMinAccel = -60;
    static double scoreCycleGrabMaxAccel = 60;
    static double scoreCycleGrabMaxWheelVel = 70;
    static double scoreCycleMaxWheelVel = 80;
    static double spinnyMaxAngVel = Math.PI * 3.5;

    static double sweepMaxWheelVel = 50;
    static double sweepMinAccel = -40;
    static double sweepMaxAccel = 40;

    static Pose2d beginBucketPose = new Pose2d(40, 66.5, Math.toRadians(180));

    static Pose2d scoreBucketPose = new Pose2d(55.5, 57, Math.toRadians(225));

    static Pose2d firstSpikeBucketPose = new Pose2d(48.5, 43, Math.toRadians(270));

    static Pose2d secondSpikeBucketPose = new Pose2d(57.25, 43, Math.toRadians(270));

    static Pose2d thirdSpikeBucketPose = new Pose2d(54, 27.25, Math.toRadians(0));

    static Pose2d parkBucketPose = new Pose2d(20, 12, Math.toRadians(180));




    public static Pose2d shiftPoseByInputs(Pose2d original, double xShift, double yShift, double degShift) {
        return new Pose2d(original.position.x+xShift,
                original.position.y+yShift,
                original.heading.toDouble()+Math.toRadians(degShift));
    }
}
