package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous(name = "Blue 6+0 Park Grab")
public class Blue6Plus0ParkGrab extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Limelight limelight;
    AutoActions autoActions;
    Sleeper sleeper;

    double horizTargetSub;
    //boolean isBlue = true;
    //double inputtedRoll;
    AutoSamplePose inputtedPose;


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
        //bart.output.sendVerticalSlidesToTarget();

        int timeToDropClipMilliseconds = 100;
        int timeToGrabClipMilliseconds = 100;
        GamepadEx playerTwo = new GamepadEx(gamepad2);
        int currentlyInputting = 0;//0=x, 1=y, 2=roll
        //double inputtedX = 0;
        //double inputtedY = 12;
        //inputtedRoll = 0;



        double yMax = 20;
        double yMin = 6;//can't reach without driving forward a little
        //previous yMax16, yMin
        inputtedPose = new AutoSamplePose(1, 0, 12, 0,
                true, true, true, 5, -5, yMax, yMin, 90, -45);

        while (inputtedPose.stillInputting && opModeInInit() && !isStopRequested()) {
            playerTwo.readButtons();

            inputtedPose.inputAutoSamplePose(playerTwo);

            telemetry.addLine(inputtedPose.toString());
            telemetry.addLine("\nPress A to Advance");
            telemetry.update();
        }

        horizTargetSub = yMax-inputtedPose.getY();//2 min

        bart.writeAllComponents();




        telemetry.addLine("DONE INPUTTING\nFINAL VALUES");
        //telemetry.addData("COLOR", isBlue ? "BLUE" : "RED");
        /*telemetry.addData("X", i);
        telemetry.addData("Y", inputtedY);
        telemetry.addData("ROLL", inputtedRoll);*/
        //telemetry.addData("horizTargetSub", horizTargetSub);
        telemetry.addLine(inputtedPose.toString());
        telemetry.update();



        //LIMELIGHT STUFF
        limelight = new Limelight(hardwareMap, inputtedPose.getColor());

        autoActions = new AutoActions(bart, drive, limelight);



        //SCORE POSES
        Pose2d scorePoseWithInput = new Pose2d(inputtedPose.getX(), 35, Math.toRadians(270));

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

                .splineToConstantHeading(scorePoseWithInput.position, Math.toRadians(270),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.preloadMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.preloadMinAccel, AutoPoses.preloadMaxAccel)
                );

        /*TrajectoryActionBuilder fromScoreToNormalizedGrab = drive.actionBuilder(scorePose)
                .strafeToConstantHeading(normalizedGrabPose.position);
*/

        TrajectoryActionBuilder fromScoreToDrop = drive.actionBuilder(scorePoseWithInput)
                .splineToConstantHeading(new Vector2d(scorePoseWithInput.position.x, scorePoseWithInput.position.y + 8), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                )
                .splineToLinearHeading(AutoPoses.dropSubClipsPose, Math.toRadians(180),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );


        TrajectoryActionBuilder fromSubDropToSpike1 = drive.actionBuilder(AutoPoses.dropSubClipsPose)
                //SPIKE 1
                .turnTo(AutoPoses.grabFrSpark1ClipsPose.heading,
                        new TurnConstraints(2.5*Math.PI, -1.5*Math.PI, 1.5*Math.PI)
                );

        TrajectoryActionBuilder fromSpike1ToDrop = drive.actionBuilder(AutoPoses.grabFrSpark1ClipsPose)
                //SPIKE 1
                .strafeToLinearHeading(AutoPoses.dropFrSpark1ClipsPose.position, AutoPoses.dropFrSpark1ClipsPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 2)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                );

        TrajectoryActionBuilder fromDropToSpike2 = drive.actionBuilder(AutoPoses.dropFrSpark1ClipsPose)
                //SPIKE 2
                .turnTo(AutoPoses.grabFrSpark2ClipsPose.heading,
                        new TurnConstraints(2.5*Math.PI, -1.5*Math.PI, 1.5*Math.PI)
                );

        TrajectoryActionBuilder fromSpike2ToDrop = drive.actionBuilder(AutoPoses.grabFrSpark2ClipsPose)
                //SPIKE 2
                .strafeToLinearHeading(AutoPoses.dropFrSpark2ClipsPose.position, AutoPoses.dropFrSpark2ClipsPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 2)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                );

        TrajectoryActionBuilder fromDropToSpike3 = drive.actionBuilder(AutoPoses.dropFrSpark2ClipsPose)
                //SPIKE 3
                .strafeToLinearHeading(AutoPoses.grabFrSpark3ClipsPose.position, AutoPoses.grabFrSpark3ClipsPose.heading,new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 2)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );

        TrajectoryActionBuilder fromSpike3ToWallGrab = drive.actionBuilder(AutoPoses.grabFrSpark3ClipsPose)
                //SPIKE 3
                .splineToSplineHeading(AutoPoses.shiftPoseByInputs(AutoPoses.grabFrSpark3ClipsPose, 2, 2, 0), Math.toRadians(45),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 2)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                )
                .splineToSplineHeading(AutoPoses.grabWallClipsPose, Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 2)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
                );

        TrajectoryActionBuilder fromGrabToScoreCycle = drive.actionBuilder(AutoPoses.grabWallClipsPose)
                .strafeToConstantHeading(AutoPoses.scoreCycleClipsPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );//26.07 first grab

        TrajectoryActionBuilder fromScoreCycleToPark = drive.actionBuilder(AutoPoses.scoreCycleClipsPose)
                .splineToSplineHeading(AutoPoses.shiftPoseByInputs(AutoPoses.scoreCycleClipsPose, 0, 1, 0), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel+20),
                                new AngularVelConstraint(Math.PI * 1.5*3)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel-20, AutoPoses.scoreCycleMaxAccel+20)
                )
                .splineToSplineHeading(new Pose2d(-27, 50, Math.toRadians(135)), Math.toRadians(135),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel+20),
                                new AngularVelConstraint(Math.PI * 1.5*3)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel-20, AutoPoses.scoreCycleMaxAccel+20)
                );

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



        telemetry.addLine("READY TO START!\n");
        /*telemetry.addData("X", inputtedX);
        telemetry.addData("Y", inputtedY);
        telemetry.addData("ROLL", inputtedRoll);*/
        telemetry.addLine(inputtedPose.toString());
        telemetry.addData("pipeline", limelight.getPipeline());
        telemetry.addData("horizTargetSub", horizTargetSub);
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
                                //fromStartToScore.build(),

                                new ParallelAction(
                                        fromStartToScore.build(),
                                        new SequentialAction(
                                                autoActions.waitTillVerticalPastInches(4, true),
                                                autoActions.setIntakeArmPosition("limelight"),
                                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                                autoActions.extendHoriz(horizTargetSub)
                                        )
                                ),



                                autoActions.openGripper(),
                                //sleeper.sleep(100),
                                autoActions.lowerOutOfWay(),
                                //sleeper.sleep(100),
                                //grab the sub sample
//                                fromScoreToNormalizedGrab.build(),
                                autoActions.lineUpWithLimelight(inputtedPose, 750),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(200),//200,150
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(300),//500,200
                                autoActions.setIntakeArmPosition("postGrab"),
                                //drop the sub sample
                                new ParallelAction(
                                        fromScoreToDrop.build(),
                                        new SequentialAction(
                                                autoActions.extendHoriz(0),
                                                autoActions.waitTillPastAngle(180, false),
                                                //autoActions.partiallyOpenGate(),
                                                autoActions.extendHoriz(11)
                                        )
                                ),
                                autoActions.setIntakeGripperOpen(true),
                                sleeper.sleep(100),
                                autoActions.setIntakeArmPosition("limelight"),
                                autoActions.setIntakeRoll(45),
                                //autoActions.extendHoriz(11),

                                //spike 1
                                fromSubDropToSpike1.build(),

                                autoActions.lineUpWithLimelight(inputtedPose, 250),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(45),
                                sleeper.sleep(200),//200,150
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(45),
                                sleeper.sleep(300),//500,200
                                autoActions.setIntakeArmPosition("postGrab"),

                                fromSpike1ToDrop.build(),

                                autoActions.setIntakeGripperOpen(true),
                                sleeper.sleep(100),
                                autoActions.setIntakeArmPosition("limelight"),
                                autoActions.setIntakeRoll(45),

                                //spike 2
                                fromDropToSpike2.build(),

                                autoActions.lineUpWithLimelight(inputtedPose, 250),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(45),
                                sleeper.sleep(200),//200,150
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(45),
                                sleeper.sleep(300),//500,200
                                autoActions.setIntakeArmPosition("postGrab"),

                                fromSpike2ToDrop.build(),

                                autoActions.setIntakeGripperOpen(true),
                                sleeper.sleep(100),
                                autoActions.setIntakeArmPosition("limelight"),
                                autoActions.setIntakeRoll(45),

                                //spike 3
                                fromDropToSpike3.build(),

                                autoActions.lineUpWithLimelight(inputtedPose, 250),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(45),
                                sleeper.sleep(200),//200,150
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(45),
                                sleeper.sleep(300),//500,200
                                autoActions.setIntakeArmPosition("postGrab"),

                                new ParallelAction(
                                        fromSpike3ToWallGrab.build(),
                                        new SequentialAction(
                                                autoActions.extendHoriz(6),
                                                autoActions.waitTillPastAngle(200, false),
                                                autoActions.setIntakeGripperOpen(true),
                                                //sleeper.sleep(100),
                                                autoActions.setIntakeArmPosition("rest"),
                                                sleeper.sleep(100),
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
                                        new SequentialAction(
                                                autoActions.waitTillPastY(52, false),
                                                autoActions.setIntakeArmPosition("parkClips"),
                                                autoActions.extendHoriz(12)
                                        ),
                                        fromGrabToScoreCycle.build()
                                ),

                                autoActions.openGripper(),

                                //autoActions.lowerToGrab(),
                                //sleeper.sleep(timeToDropClipMilliseconds),


                                autoActions.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),

                                fromScoreCycleToPark.build(),

                                //sleeper.sleep(150),

                                //autoActions.lowerToPark(),
                                //fromScoreCycleToPark.build(),



                                //3.58, 4.61
                                //1.03, 36 i



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