package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Arrays;

public class AutoPoses {

    static Pose2d beginPoseClips = new Pose2d(-5.5, 63.75, Math.toRadians(270));


    static Pose2d scorePreloadClipsPose = new Pose2d(-3.5, 35, Math.toRadians(270));

    static Pose2d dropSubClipsPose = new Pose2d(-33, 44, Math.toRadians(120));

    static Pose2d grabSpark1ClipsPose = new Pose2d(-31, 40, Math.toRadians(240));//-36,36
    static Pose2d grabSpark2ClipsPose = new Pose2d(-42, 40, Math.toRadians(250));//-41, 40
    static Pose2d grabSpark3ClipsPose = new Pose2d(-50, 26, Math.toRadians(180));//24

    static Pose2d grabSpark3ClipsPoseFirst = new Pose2d(-45, 32, Math.toRadians(235));

    static Pose2d dropSpark1ClipsPose = new Pose2d(grabSpark2ClipsPose.position, Math.toRadians(130));
    static Pose2d dropSpark2ClipsPose = new Pose2d(-43, 40, Math.toRadians(160));

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



    static Pose2d beginBucketPose = new Pose2d(39, 65.5, Math.toRadians(180));

    static Pose2d scoreBucketPose = new Pose2d(55, 57.5, Math.toRadians(225));//55.5, 57

    static Pose2d firstSpikeBucketPose = new Pose2d(48.5, 43, Math.toRadians(270));

    static Pose2d secondSpikeBucketPose = new Pose2d(57.25, 43, Math.toRadians(270));

    static Pose2d thirdSpikeBucketPose = new Pose2d(54, 27.25, Math.toRadians(0));

    static Pose2d parkBucketPose = new Pose2d(20, 12, Math.toRadians(180));




    public static Pose2d shiftPoseByInputs(Pose2d original, double xShift, double yShift, double degShift) {
        return new Pose2d(original.position.x+xShift,
                original.position.y+yShift,
                original.heading.toDouble()+Math.toRadians(degShift));
    }


    static double intakeMaxWheelVel = 55;
    static double intakeMinAccel = -60;
    static double intakeMaxAccel = 45;

    static double bucketMaxWheelVel = 70;
    static double bucketMinAccel = -65;
    static double bucketMaxAccel = 55;

    public static TrajectoryActionBuilder fromBucketToIntake(MecanumDrive drive, Pose2d intakePose) {
        return drive.actionBuilder(scoreBucketPose)
                /*.splineToLinearHeading(new Pose2d(46, 36, Math.toRadians(270)), Math.toRadians(270),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel)
                )*/
                .splineToLinearHeading(shiftPoseByInputs(intakePose, 6, 0, 0), Math.toRadians(180),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel)
                );
    }

    public static TrajectoryActionBuilder fromIntakeToBucket(MecanumDrive drive, Pose2d intakePose) {
        return drive.actionBuilder(intakePose)
                .splineToLinearHeading(shiftPoseByInputs(intakePose, 1, 0, 0), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(bucketMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(bucketMinAccel, bucketMaxAccel)
                )
                .splineToSplineHeading(new Pose2d(46, 28, Math.toRadians(270)), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel)
                )
                .splineToSplineHeading(scoreBucketPose, Math.toRadians(45),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(bucketMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(bucketMinAccel, bucketMaxAccel));
    }
}
