package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        Pose2d beginPose = new Pose2d(-5.5, 63.75, Math.toRadians(90));


        Vector2d scoreVector = new Vector2d(beginPose.position.x, 36.5);
        double scoreAngleRad = Math.toRadians(90);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);

        Vector2d score2Vector = new Vector2d(scoreVector.x+3, scoreVector.y);
        Pose2d score2Pose = new Pose2d(score2Vector, scoreAngleRad);

        Vector2d score3Vector = new Vector2d(score2Vector.x+3, scoreVector.y);
        Pose2d score3Pose = new Pose2d(score3Vector, scoreAngleRad);

        //GRAB POSE
        Vector2d grabVector = new Vector2d(-35, 60.75);//61.5ish for strafe//-32 x sometimes
        double grabAngleRad = Math.toRadians(90);
        Pose2d grabPose = new Pose2d(grabVector, grabAngleRad);

        MeepMeep meepMeep = new MeepMeep(600);
        System.setProperty("sun.java2d.opengl", "true");

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 35, Math.toRadians(180), Math.toRadians(180), 10.82)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-5.5, 63.75, Math.toRadians(90)))

                        //preload and score
                        .splineToConstantHeading(new Vector2d(-5.4, 36.5),Math.toRadians(90))
                        //push spike one
//                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-30, 40), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-40, 18), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-45, 60), Math.toRadians(90))

                        //push spike two
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-50, 18), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-55, 60), Math.toRadians(90))


                        //push spike three
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-55, 18), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-65, 50), Math.toRadians(90))

                        //pick up specimen
                        .splineToLinearHeading(new Pose2d(-35, 60.25, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0.25)

                        //score specimen
                        //.splineToConstantHeading(new Vector2d(-5.4, 36.5), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(grabVector.x, grabVector.y-2), Math.toRadians(270))
                //.splineToConstantHeading(new Vector2d(scoreVector.x-8, scoreVector.y+6), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(scoreVector.x, scoreVector.y), Math.toRadians(-90))
                        //pick up specimen 2
                        .splineToLinearHeading(new Pose2d(-35, 60.25, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0.25)

                        //score specimen
                        .splineToConstantHeading(new Vector2d(-5.4, 36.5), Math.toRadians(90))

                        //pick up specimen 3
                        .splineToLinearHeading(new Pose2d(-35, 60.25, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0.25)

                        //score specimen
                        .splineToConstantHeading(new Vector2d(-5.4, 36.5), Math.toRadians(90))

                        //pick up specimen 4
                        .splineToLinearHeading(new Pose2d(-35, 60.25, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0.25)

                        //score specimen
                        .splineToConstantHeading(new Vector2d(-5.4, 36.5), Math.toRadians(90))


                        //park
                        .splineToLinearHeading(new Pose2d(-35, 60.25, Math.toRadians(90)), Math.toRadians(90))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}