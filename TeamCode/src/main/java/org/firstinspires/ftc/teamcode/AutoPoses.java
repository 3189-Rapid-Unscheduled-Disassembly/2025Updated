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
}
