package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;

public class AutoPoses {

    static Pose2d beginPoseClips = new Pose2d(-5.5, 63.75, Math.toRadians(270));


    static Pose2d scorePreloadClipsPose = new Pose2d(-3.5, 35, Math.toRadians(270));

    static Pose2d dropSubClipsPose = new Pose2d(-31, 45, Math.toRadians(140));

    static Pose2d grabSpark1ClipsPose = new Pose2d(-31, 40, Math.toRadians(220));//-36,36
    static Pose2d grabSpark2ClipsPose = new Pose2d(-44, 40, Math.toRadians(230));//-42, 40
    static Pose2d grabSpark3ClipsPose = new Pose2d(-46, 26, Math.toRadians(180));//24

    static Pose2d grabSpark3ClipsPoseFirst = new Pose2d(-45, 32, Math.toRadians(225));//235

    static Pose2d dropSpark1ClipsPose = new Pose2d(grabSpark2ClipsPose.position.x, 43, Math.toRadians(120));

    static Pose2d dropSpark1ClipsPoseSpecial = new Pose2d(grabSpark2ClipsPose.position.x, 44, Math.toRadians(130));

    static Pose2d dropSpark2ClipsPose = new Pose2d(-44, 44, Math.toRadians(150));//-42

    static Pose2d dropSpark2ClipsPoseSpecial = new Pose2d(-31, 44, Math.toRadians(130));

    static Pose2d grabFrSpark1ClipsPose = new Pose2d(-31, 45, Math.toRadians(225));//-36,36
    static Pose2d grabFrSpark2ClipsPose = new Pose2d(-41, 45, Math.toRadians(225));//-41, 40
    static Pose2d grabFrSpark3ClipsPose = new Pose2d(-51.75, 43.75, Math.toRadians(225));//-51.5, 43.5y, 225

    //static Pose2d grabFrSpark3ClipsPoseFirst = new Pose2d(-45, 32, Math.toRadians(225));//235

    static Pose2d dropFrSpark1ClipsPose = new Pose2d(grabFrSpark2ClipsPose.position, Math.toRadians(140));
    static Pose2d dropFrSpark2ClipsPose = new Pose2d(-31, 45, Math.toRadians(140));

    static Pose2d grabWallClipsPose = new Pose2d(-41, 60, Math.toRadians(90));//61

    //DO NOT USE
    static Pose2d scoreCycleClipsPose = new Pose2d(-2, 30.25, Math.toRadians(90));//30.25, -5
    //DO NOT USE

    static Pose2d scoreCycleClipsPoseSpecial = new Pose2d(-2, 28, Math.toRadians(90));//30.25, 29, 28.5, -5x

    static Pose2d grabWallClipsTeleopPose = shiftPoseByInputs(grabWallClipsPose, -1, 1, 0);
    static Pose2d scoreCycleClipsTeleopPose = shiftPoseByInputs(scoreCycleClipsPose, 0, 0, 0);


    static Pose2d clipsParkPose = shiftPoseByInputs(grabWallClipsPose, -6, -14, 0);

    static Pose2d spike1Short = new Pose2d(53.5, 52, Math.toRadians(252));//53.5x, 52.75
    static Pose2d spike2Short = new Pose2d(57.25+0.25, 52, Math.toRadians(270));//57, 52.5
    static Pose2d spike3Short = new Pose2d(54.25, 49, Math.toRadians(290));//300, 53.75


    static double preloadMinAccel = -70;
    static double preloadMaxAccel = 70;
    static double preloadMaxWheelVel = 70;

    static double scoreCycleMinAccel = -85;//-75
    static double scoreCycleMaxAccel = 85;//70
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

    static Pose2d scoreBucketCyclePose = new Pose2d(55+1, 46-1, Math.toRadians(240));//59, 50, 250

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


    //this is what we will use for all autos and stuff. we only need the the other one to deal with player inputted x shifts
    public static TrajectoryActionBuilder fromGrabToScoreCycle(MecanumDrive drive) {
        return fromGrabToScoreCycle(drive,0);
    }
    public static TrajectoryActionBuilder fromGrabToScoreCycle(MecanumDrive drive, double grabWallClipsXShift) {
        Pose2d grabWallClipsNow = shiftPoseByInputs(grabWallClipsPose, grabWallClipsXShift, 0, 0);
        Pose2d scoreCycleClipsPoseNow = shiftPoseByInputs(scoreCycleClipsPoseSpecial, grabWallClipsXShift, 0, 0);
        return drive.actionBuilder(grabWallClipsNow)
                .strafeToConstantHeading(scoreCycleClipsPoseNow.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );//26.07 first grab
    }
    /*public static TrajectoryActionBuilder fromGrabToScoreCycleTeleop(MecanumDrive drive) {
        return drive.actionBuilder(grabWallClipsTeleopPose)
                .strafeToConstantHeading(scoreCycleClipsTeleopPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );//26.07 first grab
    }*/

    public static TrajectoryActionBuilder fromScoreCycleToGrab(MecanumDrive drive) {
        return fromScoreCycleToGrab(drive, 0);
    }
    public static TrajectoryActionBuilder fromScoreCycleToGrab(MecanumDrive drive, double grabWallClipsXShift) {
        Pose2d grabWallClipsNow = shiftPoseByInputs(grabWallClipsPose, grabWallClipsXShift, 0, 0);
        Pose2d scoreCycleClipsPoseNow = shiftPoseByInputs(scoreCycleClipsPoseSpecial, grabWallClipsXShift, 0, 0);
        return drive.actionBuilder(scoreCycleClipsPoseNow)
                .strafeToConstantHeading(grabWallClipsNow.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleGrabMinAccel, AutoPoses.scoreCycleGrabMaxAccel)
                );
    }
    /*public static TrajectoryActionBuilder fromScoreCycleToGrabTeleop(MecanumDrive drive) {
        return drive.actionBuilder(scoreCycleClipsTeleopPose)
                .strafeToConstantHeading(grabWallClipsTeleopPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleGrabMinAccel, AutoPoses.scoreCycleGrabMaxAccel)
                );
    }*/

    public static TrajectoryActionBuilder fromBucketToIntake(MecanumDrive drive, Pose2d bucketPose, Pose2d intakePose) {
        return drive.actionBuilder(bucketPose)
                /*.splineToLinearHeading(shiftPoseByInputs(intakePose, 18, 6, 0), Math.toRadians(210),
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

    //currently assume 6 inches off sub
    public static TrajectoryActionBuilder fromBucketToIntakeOffSub(MecanumDrive drive, Pose2d bucketPose, AutoSamplePose samplePose) {
        double intakeX = intakeXFromSamplePoseOffSub(samplePose);
        Pose2d intakePose = new Pose2d(intakeX, samplePose.getY(), Math.toRadians(180));
        return drive.actionBuilder(bucketPose)
                /*.splineToSplineHeading(new Pose2d(49, 23, Math.toRadians(180)), Math.toRadians(225),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel)
                )*/
                .splineToSplineHeading(intakePose, Math.toRadians(180),
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
                .splineToSplineHeading(scoreBucketCyclePose, Math.toRadians(60),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(bucketMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(bucketMinAccel, bucketMaxAccel));
    }

    public static TrajectoryActionBuilder fromIntakeToBucketOffSub(MecanumDrive drive, AutoSamplePose samplePose) {
        double intakeX = intakeXFromSamplePoseOffSub(samplePose);
        Pose2d intakePose = new Pose2d(intakeX, samplePose.getY(), Math.toRadians(180));

        return drive.actionBuilder(intakePose)
                .splineToLinearHeading(shiftPoseByInputs(intakePose, 1, 0, 0), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(bucketMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(bucketMinAccel, bucketMaxAccel)
                )
                .splineToSplineHeading(scoreBucketCyclePose, Math.toRadians(70),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(bucketMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(bucketMinAccel, bucketMaxAccel));
    }

    private static double intakeXFromSamplePoseOffSub(AutoSamplePose samplePose) {
        double sampleX = samplePose.getX();
        double sampleXMax = samplePose.xMax;
        double xDiff = sampleXMax - sampleX;
        double maxGrabX = 23+6;
        return maxGrabX - xDiff;
    }
}