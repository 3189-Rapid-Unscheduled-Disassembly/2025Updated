package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous(name = "Blue 5+0 Dry")
public class Blue5Plus0Dry extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    AutoActions autoActions;
    Sleeper sleeper;



    //OUTPUT SYNCHRONOUS MOVEMENTS ACTIONS IF NEEDED


    @Override
    public void runOpMode() throws InterruptedException {
        //create robot 6 5/16 from wall   x = 32 from wall
        //Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));



        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, AutoPoses.beginPoseClips);
        sleeper = new Sleeper();

        //horiz target
        //58, 53, 240deg(60deg)
        //58, 48, 270deg






        //bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.readHubs();
        //bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, -40, 90, false));
        bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
        //bart.intake.intakeArm.setRollDeg(45);
        bart.intake.fullyOpenGate();
        bart.writeAllComponents();
        //bart.output.sendVerticalSlidesToTarget();

        int timeToDropClipMilliseconds = 100;
        int timeToGrabClipMilliseconds = 100;

        autoActions = new AutoActions(bart, drive);


        //DRIVE TRAJECTORIES
        //TrajectoryActionBuilder fromStartToScore;
        //if left of x -5, we shift over first
        /*if (inputtedX > -5) {
            fromStartToScore = drive.actionBuilder(beginPose)
                    .strafeToConstantHeading(new Vector2d(inputtedX, beginPose.position.y))
                    .strafeToConstantHeading(scoreVector);
        } else {
            fromStartToScore = drive.actionBuilder(beginPose)
                    .strafeToConstantHeading(scoreVector);
        }*/
        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(AutoPoses.beginPoseClips)

                .splineToConstantHeading(AutoPoses.scorePreloadClipsPose.position, Math.toRadians(270),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.preloadMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.preloadMinAccel, AutoPoses.preloadMaxAccel)
                );

        /*TrajectoryActionBuilder fromScoreToNormalizedGrab = drive.actionBuilder(scorePose)
                .strafeToConstantHeading(normalizedGrabPose.position);
*/


        TrajectoryActionBuilder fromPreloadScoreToSweep = drive.actionBuilder(AutoPoses.scorePreloadClipsPose)
                //SPIKE 1
                .strafeToLinearHeading(AutoPoses.shiftPoseByInputs(AutoPoses.scorePreloadClipsPose,0, 6, -1000).position, AutoPoses.grabSpark1ClipsPose.heading.toDouble())
                .strafeToLinearHeading(AutoPoses.grabSpark1ClipsPose.position, AutoPoses.grabSpark1ClipsPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 2)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                )
                /*.turnTo(grabSpark1Pose.heading.toDouble(),
                        new TurnConstraints(Math.PI*1.5, -Math.PI*1.8, Math.PI*1.8)
                )*/
                .strafeToLinearHeading(AutoPoses.dropSpark1ClipsPose.position, AutoPoses.dropSpark1ClipsPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                )
                //SPIKE 2
                .turnTo(AutoPoses.grabSpark2ClipsPose.heading.toDouble(),
                        new TurnConstraints(Math.PI*1.5, -Math.PI*1.8, Math.PI*1.8)
                )
                .strafeToLinearHeading(AutoPoses.dropSpark2ClipsPose.position, AutoPoses.dropSpark2ClipsPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                )
                //SPIKE 3
                .splineToLinearHeading(AutoPoses.grabSpark3ClipsPose, Math.toRadians(180),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                )
                /*
                .splineToSplineHeading(new Pose2d(grabSpark3Pose.position.x, grabSpark3Pose.position.y+6, grabSpark3Pose.heading.toDouble()), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)
                )*/
                /*
                .splineToSplineHeading(new Pose2d(grabSpark3Pose.position.x, 40, Math.toRadians(90)), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)
                )*/
                /*.splineToSplineHeading(new Pose2d(grabVector.x-5, grabVector.y - 12, Math.toRadians(90)), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)
                )*/
                .splineToSplineHeading(new Pose2d(AutoPoses.grabSpark3ClipsPose.position.x+0.5, AutoPoses.grabSpark3ClipsPose.position.y+6, Math.toRadians(160)), Math.toRadians(80),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                )
                /*.splineToSplineHeading(new Pose2d(grabSpark3Pose.position.x+8, grabSpark3Pose.position.y+10, Math.toRadians(135)), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(sweepMinAccel, sweepMaxAccel)
                )*/
                .splineToSplineHeading(AutoPoses.grabWallClipsPose, Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel-5),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel+5, AutoPoses.sweepMaxAccel-5)
                );



        TrajectoryActionBuilder fromGrabToScoreCycle = drive.actionBuilder(AutoPoses.grabWallClipsPose)
                .strafeToConstantHeading(AutoPoses.scoreCycleClipsPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );//26.07 first grab
        //last score: 38.52
        //press play: 11.71

        TrajectoryActionBuilder fromScoreCycleToGrab = drive.actionBuilder(AutoPoses.scoreCycleClipsPose)
                .strafeToConstantHeading(AutoPoses.grabWallClipsPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleGrabMinAccel, AutoPoses.scoreCycleGrabMaxAccel)
                );
                /*
                .splineToConstantHeading(new Vector2d(scoreCycleVector.x, scoreCycleVector.y+2), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleMaxAccel)
                )
                .splineToConstantHeading(new Vector2d(grabVector.x+4, grabVector.y-5), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleMaxAccel)
                )
                .splineToConstantHeading(grabVector, Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleMaxAccel)
                );
*/


        telemetry.addLine("READY TO START!\n");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        //SEND COMPONENTS TO POSITION EVERY FRAME
                        autoActions.readComponents(),
                        autoActions.sendComponentsToPositions(),
                        autoActions.writeComponents(),
                        new SequentialAction(

                                //SCORE PRELOAD
                                autoActions.raiseToHighBarFront(),
                                fromStartToScore.build(),

                                autoActions.openGripper(),
                                autoActions.lowerOutOfWay(),

                                autoActions.closeGate(),

                                //SWEEP
                                new ParallelAction(
                                        fromPreloadScoreToSweep.build(),
                                        new SequentialAction(
                                                //SPIKE 1
                                                autoActions.waitTillPastX(-24, false),
                                                autoActions.extendHoriz(13),
                                                //SPIKE 2
                                                autoActions.waitTillPastAngle(140, false),
                                                autoActions.setGateOncePastAngle(140, true, 1),
                                                autoActions.setGateOncePastAngle(220, true, 0),
                                                //SPIKE 3
                                                autoActions.waitTillPastAngle(140, false),
                                                autoActions.setGateOncePastAngle(140,  true, 1),
                                                autoActions.waitTillPastY(26, false),
                                                autoActions.closeGate(),
                                                autoActions.waitTillPastY(36, true),
                                                autoActions.fullyOpenGate(),
                                                //autoActions.waitTillPastAngle(145, false),
                                                autoActions.extendHoriz(0),

                                                autoActions.waitTillHorizIsRetracted(), autoActions.lowerToGrab()

                                        )
                                ),






                                //CYCLE
                                //grab clip 1
                                autoActions.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //autoActions.aboveGrab(),

                                //SCORE THE CLIP
                                new ParallelAction(
                                        autoActions.raiseToHighBarBackOnceAwayFromWall(),
                                        autoActions.closeGripperTightAfterDelay(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 3
                                autoActions.openGripper(),
                                //autoActions.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                //grab
                                autoActions.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
                                fromScoreCycleToGrab.build(),
                                autoActions.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        autoActions.raiseToHighBarBackOnceAwayFromWall(),
                                        autoActions.closeGripperTightAfterDelay(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 4
                                //grab
                                autoActions.openGripper(),
                                //autoActions.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                autoActions.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
                                fromScoreCycleToGrab.build(),
                                autoActions.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        autoActions.raiseToHighBarBackOnceAwayFromWall(),
                                        autoActions.closeGripperTightAfterDelay(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 5
                                //grab
                                autoActions.openGripper(),
                                //autoActions.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                autoActions.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
                                fromScoreCycleToGrab.build(),
                                autoActions.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        autoActions.raiseToHighBarBackOnceAwayFromWall(),
                                        autoActions.closeGripperTightAfterDelay(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 6
                                //grab
                                autoActions.openGripper(),
                                //autoActions.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                autoActions.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
                                fromScoreCycleToGrab.build(),
                                autoActions.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        autoActions.raiseToHighBarBackOnceAwayFromWall(),
                                        autoActions.closeGripperTightAfterDelay(),
                                        autoActions.lowerIntakeAtEnd(),
                                        fromGrabToScoreCycle.build()
                                ),

                                autoActions.openGripper(),

                                //autoActions.lowerToGrab(),
                                //sleeper.sleep(timeToDropClipMilliseconds),


                                autoActions.moveWristOutOfWay(),
                                //sleeper.sleep(150),

                                //autoActions.lowerToPark(),
                                //fromScoreCycleToPark.build(),



                                //3.58, 4.61
                                //1.03, 36 in


                                sleeper.sleep(1000),

                                autoActions.endProgram()
                        )
                )
        );

        //put the horizontal slide back home
        /*while (!bart.intake.isAtSavedPosition("transfer") && opModeIsActive()) {
            bart.intake.setHorizontalSlideToSavedPosition("transfer");
        }
        bart.intake.horizontalSlide.setPower(0);*/


    }
}