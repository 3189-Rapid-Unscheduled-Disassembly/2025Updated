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


        Pose2d dropSubPose = new Pose2d(-30, 40, Math.toRadians(120));

        Pose2d grabSpark1Pose = new Pose2d(-32, 40, Math.toRadians(240));//-36,36
        Pose2d grabSpark2Pose = new Pose2d(-42, 40, Math.toRadians(240));//-41, 40
        Pose2d grabSpark3Pose = new Pose2d(-46, 26, Math.toRadians(200));




        Pose2d dropSpark1Pose = new Pose2d(grabSpark2Pose.position, Math.toRadians(130));

        Pose2d dropSpark2Pose = new Pose2d(-44, 40, Math.toRadians(150));




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


        myBot.runAction(myBot.getDrive().actionBuilder(grabWallClipsPose)
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