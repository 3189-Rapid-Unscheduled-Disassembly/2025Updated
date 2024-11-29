package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
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
                        .splineToConstantHeading(new Vector2d(-5.4, 36.5), Math.toRadians(90))

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