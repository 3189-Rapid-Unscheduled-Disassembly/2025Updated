package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.Arrays;

public class AutoPoses {

    static Pose2d beginPoseClips = new Pose2d(-5.5, 63.75, Math.toRadians(270));


    static Pose2d scorePreloadClipsPose = new Pose2d(-3.5, 35, Math.toRadians(270));

    static Pose2d dropSubClipsPose = new Pose2d(-31, 45, Math.toRadians(140));

    static Pose2d grabSpark1ClipsPose = new Pose2d(-31, 40, Math.toRadians(220));//-36,36
    static Pose2d grabSpark2ClipsPose = new Pose2d(-42, 40, Math.toRadians(240));//-41, 40
    static Pose2d grabSpark3ClipsPose = new Pose2d(-46, 26, Math.toRadians(180));//24

    static Pose2d grabSpark3ClipsPoseFirst = new Pose2d(-45, 32, Math.toRadians(225));//235

    static Pose2d dropSpark1ClipsPose = new Pose2d(grabSpark2ClipsPose.position.x, 43, Math.toRadians(120));
    static Pose2d dropSpark2ClipsPose = new Pose2d(-43, 40, Math.toRadians(120));

    static Pose2d dropSpark2ClipsPoseSpecial = new Pose2d(-31, 42, Math.toRadians(120));

    static Pose2d grabFrSpark1ClipsPose = new Pose2d(-31, 45, Math.toRadians(225));//-36,36
    static Pose2d grabFrSpark2ClipsPose = new Pose2d(-41, 45, Math.toRadians(225));//-41, 40
    static Pose2d grabFrSpark3ClipsPose = new Pose2d(-52, 45, Math.toRadians(225));//24

    //static Pose2d grabFrSpark3ClipsPoseFirst = new Pose2d(-45, 32, Math.toRadians(225));//235

    static Pose2d dropFrSpark1ClipsPose = new Pose2d(grabFrSpark2ClipsPose.position, Math.toRadians(140));
    static Pose2d dropFrSpark2ClipsPose = new Pose2d(-31, 45, Math.toRadians(140));

    static Pose2d grabWallClipsPose = new Pose2d(-41, 61, Math.toRadians(90));
    static Pose2d scoreCycleClipsPose = new Pose2d(-5, 30.25, Math.toRadians(90));

    static Pose2d scoreCycleClipsPoseSpecial = new Pose2d(-5, 42, Math.toRadians(90));

    static Pose2d grabWallClipsTeleopPose = shiftPoseByInputs(grabWallClipsPose, -1, 1, 0);
    static Pose2d scoreCycleClipsTeleopPose = shiftPoseByInputs(scoreCycleClipsPose, 0, 0, 0);


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

    static double grabSpikeClipsMaxWheelVel = 40;
    static double grabSpikeClipsMinAccel = -40;
    static double grabSpikeClipsMaxAccel = 40;



    static Pose2d beginBucketPose = new Pose2d(39, 65.5, Math.toRadians(180));

    static Pose2d scoreBucketPose = new Pose2d(55, 57.5, Math.toRadians(225));//55.5, 57

    static Pose2d scoreBucketCyclePose = new Pose2d(58, 43, Math.toRadians(250));//59, 50, 250

    static Pose2d scoreBucketCycleForThirdSpikePose = new Pose2d(58, 51.5, Math.toRadians(250));

    static Pose2d firstSpikeBucketPose = new Pose2d(48.5, 43, Math.toRadians(270));

    static Pose2d secondSpikeBucketPose = new Pose2d(57.25, 43, Math.toRadians(270));

    static Pose2d thirdSpikeBucketPose = new Pose2d(54, 27.25, Math.toRadians(0));

    static Pose2d parkBucketPose = new Pose2d(20, 12, Math.toRadians(180));



    static int timeToDropClipMilliseconds = 100;
    static int timeToGrabClipMilliseconds = 100;

    public static Pose2d shiftPoseByInputs(Pose2d original, double xShift, double yShift, double degShift) {
        return new Pose2d(original.position.x+xShift,
                original.position.y+yShift,
                original.heading.toDouble()+Math.toRadians(degShift));
    }


    static double intakeMaxWheelVel = 55;
    static double intakeMinAccel = -60;
    static double intakeMaxAccel = 45;

    static double bucketMaxWheelVel = 65;//70
    static double bucketMinAccel = -50;//-65
    static double bucketMaxAccel = 55;//55

    public static TrajectoryActionBuilder fromGrabToScoreCycle(MecanumDrive drive) {
        return drive.actionBuilder(AutoPoses.grabWallClipsPose)
                .strafeToConstantHeading(AutoPoses.scoreCycleClipsPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );//26.07 first grab
    }
    public static TrajectoryActionBuilder fromGrabToScoreCycleTeleop(MecanumDrive drive) {
        return drive.actionBuilder(grabWallClipsTeleopPose)
                .strafeToConstantHeading(scoreCycleClipsTeleopPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );//26.07 first grab
    }

    public static TrajectoryActionBuilder fromScoreCycleToGrab(MecanumDrive drive) {
        return drive.actionBuilder(AutoPoses.scoreCycleClipsPose)
                .strafeToConstantHeading(AutoPoses.grabWallClipsPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleGrabMinAccel, AutoPoses.scoreCycleGrabMaxAccel)
                );
    }
    public static TrajectoryActionBuilder fromScoreCycleToGrabTeleop(MecanumDrive drive) {
        return drive.actionBuilder(scoreCycleClipsTeleopPose)
                .strafeToConstantHeading(grabWallClipsTeleopPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleGrabMinAccel, AutoPoses.scoreCycleGrabMaxAccel)
                );
    }

    public static TrajectoryActionBuilder fromBucketToIntake(MecanumDrive drive, Pose2d bucketPose, Pose2d intakePose) {
        return drive.actionBuilder(bucketPose)
                /*.splineToSplineHeading(new Pose2d(49, 23, Math.toRadians(180)), Math.toRadians(225),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel)
                )*/
                .splineToSplineHeading(shiftPoseByInputs(intakePose, 1, 0, 0), Math.toRadians(180),
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
                /*.splineToSplineHeading(new Pose2d(50, 25, Math.toRadians(250)), Math.toRadians(70),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(bucketMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(bucketMinAccel, bucketMaxAccel)
                )*/
                .splineToSplineHeading(scoreBucketCyclePose, Math.toRadians(70),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(bucketMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(bucketMinAccel, bucketMaxAccel));
    }
}
