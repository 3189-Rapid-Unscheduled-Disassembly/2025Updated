package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {

    public static void main(String[] args) {



        Vector2d scoreCycleVector = new Vector2d(-3, 32.5);
        double scoreCycleAngleRad = Math.toRadians(90);
        Pose2d scoreCyclePose = new Pose2d(scoreCycleVector, scoreCycleAngleRad);
        //GRAB POSE
        Vector2d grabVector = new Vector2d(-35, 57.25);//61.5ish for strafe//-32 x sometimes
        double grabAngleRad = Math.toRadians(90);
        Pose2d grabPose = new Pose2d(grabVector, grabAngleRad);

        Pose2d scoreCycleClipsPose = new Pose2d(-5, 30.25, Math.toRadians(90));
        Pose2d grabWallClipsPose = new Pose2d(-41, 61, Math.toRadians(90));
        Pose2d clipsParkPose = shiftPoseByInputs(grabWallClipsPose, -6, -10, 0);


        MeepMeep meepMeep = new MeepMeep(600);
        System.setProperty("sun.java2d.opengl", "true");

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 35, Math.toRadians(180), Math.toRadians(180), 10.82)
                .build();

       Pose2d grabSpark3ClipsPose = new Pose2d(-45.5, 24, Math.toRadians(200));

        Pose2d grabSpark3ClipsPoseFirst = new Pose2d(-40, 32, Math.toRadians(225));

        Pose2d scoreBucketCyclePose = new Pose2d(59, 42, Math.toRadians(240));//59, 50, 250

        Pose2d intakePose = new Pose2d(23, 12, Math.toRadians(180));

        Pose2d dropSpark2ClipsPose = new Pose2d(-43, 40, Math.toRadians(120));

        Pose2d grabFrSpark3ClipsPose = new Pose2d(-52, 45, Math.toRadians(225));//24


        myBot.runAction(myBot.getDrive().actionBuilder(scoreBucketCyclePose)
                        /*.strafeToConstantHeading(shiftPoseByInputs(scoreCycleClipsPose, 0, 6, 0).position)
                                .strafeToConstantHeading(clipsParkPose.position)(/
                //.splineToLinearHeading(grabSpark3Pose, Math.toRadians(180)

                //)
                /*.splineToConstantHeading(grabSpark3Pose.position, Math.toRadians(270),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)
                )*/
                //.splineToSplineHeading(new Pose2d(grabSpark3Pose.position.x+1, grabSpark3Pose.position.y+6, Math.toRadians(160)), Math.toRadians(80)
                  //                     )
                //.splineToSplineHeading(new Pose2d(grabSpark3Pose.position.x+4, grabSpark3Pose.position.y+10, Math.toRadians(135)), Math.toRadians(90)

                //)
                //.splineToSplineHeading(new Pose2d(grabSpark3Pose.position.x, 40, Math.toRadians(90)), Math.toRadians(0)

                //)
                /*.splineToSplineHeading(new Pose2d(grabVector.x-5, grabVector.y - 12, Math.toRadians(90)), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)
                )*/
                //.splineToSplineHeading(grabPose, Math.toRadians(90)//,
                        /*new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)*/
                //)

                /*.splineToSplineHeading(shiftPoseByInputs(grabWallClipsPose, 4, -4, 90), Math.toRadians(0)

                )
                .splineToSplineHeading(new Pose2d(48, 56.5, Math.toRadians(225)), Math.toRadians(0)*/
                //)

                /*.splineToLinearHeading(grabSpark3ClipsPoseFirst, Math.toRadians(270)
                )*/
                //.splineToConstantHeading(grabSpark3ClipsPoseFirst.position, Math.toRadians(270)
                //)
                /*.splineToSplineHeading(AutoPoses.shiftPoseByInputs(AutoPoses.grabSpark3ClipsPose, 3, -3, 0), Math.toRadians(180),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel))*/
                /*.splineToSplineHeading(grabSpark3ClipsPose, Math.toRadians(90)
                )
                .splineToSplineHeading(new Pose2d(grabSpark3ClipsPose.position.x+0.5, grabSpark3ClipsPose.position.y+6, Math.toRadians(160)), Math.toRadians(80)
                )*/
                /*.splineToLinearHeading(new Pose2d(48, 24, Math.toRadians(225+45/2)), Math.toRadians(225+45/2))
                .splineToSplineHeading(new Pose2d(23, 12, Math.toRadians(180)), Math.toRadians(180))


                .splineToLinearHeading(new Pose2d(48, 24, Math.toRadians(225+45/2)), Math.toRadians(225+45/2-180))
                .splineToSplineHeading(scoreBucketPose, Math.toRadians(45))*/
                //.splineToLinearHeading(new Pose2d(48, 42, Math.toRadians(247.5)), Math.toRadians(247.5)

                //)
                //.splineToSplineHeading(shiftPoseByInputs(intakePose, 6, 4, 0), Math.toRadians(180)
                //)
                        //.splineToLinearHeading(shiftPoseByInputs(intakePose, 4, 4, 0), Math.toRadians(180))
//                .splineToSplineHeading(intakePose, Math.toRadians(180))
                /*.splineToLinearHeading(shiftPoseByInputs(intakePose, 1, 0, 0), Math.toRadians(0))
                .splineToSplineHeading(scoreBucketPose, Math.toRadians(60))*/
                /*.splineToLinearHeading(shiftPoseByInputs(intakePose, 4, 0, 0), Math.toRadians(180)
                )
                .strafeToConstantHeading(intakePose.position)*/



                //.splineToLinearHeading(new Pose2d(48, 24, Math.toRadians(250)), Math.toRadians(250))
                //.splineToSplineHeading(new Pose2d(46, 19, Math.toRadians(225)), Math.toRadians(225))

                //.splineToSplineHeading(shiftPoseByInputs(intakePose, 0, 0, 0), Math.toRadians(180))


                //.splineToLinearHeading(shiftPoseByInputs(intakePose, 1, 0, 0), Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(40, 14, Math.toRadians(270-45)), Math.toRadians(45))
                //.splineToSplineHeading(new Pose2d(50, 24, Math.toRadians(250)), Math.toRadians(70))
                //.splineToSplineHeading(scoreBucketCyclePose, Math.toRadians(45))

                /*.splineToSplineHeading(new Pose2d(grabSpark3Pose.position.x+8, grabSpark3Pose.position.y+10, Math.toRadians(135)), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)
                )*/
                //.splineToSplineHeading(grabWallClipsPose, Math.toRadians(90)

                //)


                //.splineToSplineHeading(new Pose2d(-45, 45, grabFrSpark3ClipsPose.heading.toDouble()), Math.toRadians(180))
                .splineToLinearHeading(shiftPoseByInputs(intakePose, 12, 0, 0), Math.toRadians(180))
                .splineToSplineHeading(shiftPoseByInputs(intakePose, 1, 0, 0), Math.toRadians(180))
                //.splineToSplineHeading(scoreBucketCyclePose, scoreBucketCyclePose.heading.toDouble() - Math.PI)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static Pose2d shiftPoseByInputs(Pose2d original, double xShift, double yShift, double degShift) {
        return new Pose2d(original.position.x+xShift,
                original.position.y+yShift,
                original.heading.toDouble()+Math.toRadians(degShift));
    }
}