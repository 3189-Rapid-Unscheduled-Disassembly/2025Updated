package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        Pose2d beginPose = new Pose2d(-5.5, 63.75, Math.toRadians(270));


        Vector2d scoreVector = new Vector2d(beginPose.position.x, 32);//36.5 for backwards
        double scoreAngleRad = Math.toRadians(270);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);

        double grabY = 42;
        Vector2d grabSpark1Vector = new Vector2d(-29.5, grabY+1);
        double grabSpark1Rad = Math.toRadians(225);
        Pose2d grabSpark1Pose= new Pose2d(grabSpark1Vector, grabSpark1Rad);


        Vector2d scoreCycleVector = new Vector2d(-2, 33);
        double scoreCycleAngleRad = Math.toRadians(90);
        Pose2d scoreCyclePose = new Pose2d(scoreCycleVector, scoreCycleAngleRad);
        //GRAB POSE
        Vector2d grabVector = new Vector2d(-35, 57.25);//61.5ish for strafe//-32 x sometimes
        double grabAngleRad = Math.toRadians(90);
        Pose2d grabPose = new Pose2d(grabVector, grabAngleRad);

        MeepMeep meepMeep = new MeepMeep(600);
        System.setProperty("sun.java2d.opengl", "true");

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 35, Math.toRadians(180), Math.toRadians(180), 10.82)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(scorePose)
                        /*.strafeToConstantHeading(scoreCycleVector)
                        .waitSeconds(0.1)
                        .strafeToConstantHeading(new Vector2d(grabVector.x, grabVector.y-5))
                        .strafeToConstantHeading(grabVector)*/

                .splineToLinearHeading(new Pose2d(scoreVector.x-10, grabSpark1Vector.y, grabSpark1Rad), Math.toRadians(180))
                //.splineToConstantHeading(new Vector2d(-42, 43.5), Math.toRadians(180));//-46
                .splineToConstantHeading(grabSpark1Vector, Math.toRadians(180))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}