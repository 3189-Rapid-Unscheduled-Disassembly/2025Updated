package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous(name = "Blue 0+6 Park")
public class Blue0Plus6Park extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Limelight limelight;
    AutoActions autoActions;
    Sleeper sleeper;

    double horizTargetSub;
    double horizTargetSub2;
    double horizTargetSub3;


    AutoSamplePose inputtedPose;
    AutoSamplePose inputtedPose2;
    AutoSamplePose inputtedPose3;




    //OUTPUT SYNCHRONOUS MOVEMENTS ACTIONS IF NEEDED


    @Override
    public void runOpMode() throws InterruptedException {
        //create robot 6 5/16 from wall   x = 32 from wall
        //Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));



        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, AutoPoses.beginBucketPose);
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

        GamepadEx playerTwo = new GamepadEx(gamepad2);


        double yMax = 12;
        double yMin = 0;

        double xMax = 10;//12
        double xMin = -7;//-2
        //previous yMax16, yMin
        inputtedPose = new AutoSamplePose(0, 0, 12, 0,
                true, true, true, xMax, xMin, yMax, yMin, 90, -45);

        inputtedPose2 = new AutoSamplePose(0, 0, 12, 0,
                true, true, true, xMax, xMin, yMax, yMin, 90, -45);

        inputtedPose3 = new AutoSamplePose(0, 0, 12, 0,
                true, true, true, xMax, xMin, yMax, yMin, 90, -45);

        while (inputtedPose.stillInputting && opModeInInit() && !isStopRequested()) {
            playerTwo.readButtons();

            inputtedPose.inputAutoSamplePose(playerTwo);

            telemetry.addLine(inputtedPose.toString());
            telemetry.addLine("\nSample 1\nPress A to Advance");
            telemetry.update();
        }
        horizTargetSub = xMax-inputtedPose.getX();//2 min

        while (inputtedPose2.stillInputting && opModeInInit() && !isStopRequested()) {
            playerTwo.readButtons();

            inputtedPose2.inputAutoSamplePose(playerTwo);

            telemetry.addLine(inputtedPose2.toString());
            telemetry.addLine("\nSample 2\nPress A to Advance");
            telemetry.update();
        }
        horizTargetSub2 = xMax-inputtedPose2.getX();//2 min

        while (inputtedPose3.stillInputting && opModeInInit() && !isStopRequested()) {
            playerTwo.readButtons();

            inputtedPose3.inputAutoSamplePose(playerTwo);

            telemetry.addLine(inputtedPose3.toString());
            telemetry.addLine("\nSample 3\nPress A to Advance");
            telemetry.update();
        }
        horizTargetSub3 = xMax-inputtedPose3.getX();//2 min








        //LIMELIGHT STUFF
        limelight = new Limelight(hardwareMap, inputtedPose.getColor());

        autoActions = new AutoActions(bart, drive, limelight);

        int maxHuntingTimeMS = 750;
        int timeToDropMS = 50;

        double intakeX = 23;

        Pose2d intakePose = new Pose2d(intakeX, inputtedPose.getY(), Math.toRadians(180));
        Pose2d intakePose2 = new Pose2d(intakeX, inputtedPose2.getY(), Math.toRadians(180));
        Pose2d intakePose3 = new Pose2d(intakeX, inputtedPose3.getY(), Math.toRadians(180));


        TrajectoryActionBuilder fromBucketToIntake1 = AutoPoses.fromBucketToIntake(drive, intakePose);
        TrajectoryActionBuilder fromIntakeToBucket1 = AutoPoses.fromIntakeToBucket(drive, intakePose);

        TrajectoryActionBuilder fromBucketToIntake2 = AutoPoses.fromBucketToIntake(drive, intakePose2);
        TrajectoryActionBuilder fromIntakeToBucket2 = AutoPoses.fromIntakeToBucket(drive, intakePose2);

        TrajectoryActionBuilder fromBucketToIntake3 = AutoPoses.fromBucketToIntake(drive, intakePose3);



/*
        TrajectoryActionBuilder fromScoreToIntake = drive.actionBuilder(AutoPoses.scoreBucketPose)
                .strafeToLinearHeading(AutoPoses.shiftPoseByInputs(intakePose, 20, 0, 0).position, intakePose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                )
                .strafeToConstantHeading(intakePose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                );

        TrajectoryActionBuilder fromIntakeToScore = drive.actionBuilder(intakePose)
                .strafeToLinearHeading(AutoPoses.shiftPoseByInputs(intakePose, 20, 0, 0).position, AutoPoses.scoreBucketPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                )
                .strafeToConstantHeading(AutoPoses.scoreBucketPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                );

        TrajectoryActionBuilder fromScoreToIntake2 = drive.actionBuilder(AutoPoses.scoreBucketPose)
                .strafeToLinearHeading(AutoPoses.shiftPoseByInputs(intakePose2, 20, 0, 0).position, intakePose2.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                )
                .strafeToConstantHeading(intakePose2.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                );

        TrajectoryActionBuilder fromIntake2ToScore = drive.actionBuilder(intakePose2)
                .strafeToLinearHeading(AutoPoses.shiftPoseByInputs(intakePose2, 20, 0, 0).position, AutoPoses.scoreBucketPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                )
                .strafeToConstantHeading(AutoPoses.scoreBucketPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.intakeMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.intakeMinAccel, AutoPoses.intakeMaxAccel)
                );*/


        //static Pose2d scoreBucketPose = new Pose2d(55.5, 57, Math.toRadians(225));
        Pose2d spike1Short = new Pose2d(54, 53, Math.toRadians(250));//53.5x
        Pose2d spike2Short = new Pose2d(57.5, 52, Math.toRadians(270));
        Pose2d spike3Short = new Pose2d(54, 49.5, Math.toRadians(300));


        //SCORE POSES
        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(AutoPoses.beginBucketPose)
                .strafeToLinearHeading(AutoPoses.scoreBucketPose.position, AutoPoses.scoreBucketPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.sweepMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.sweepMinAccel, AutoPoses.sweepMaxAccel)
            );

        TrajectoryActionBuilder fromScoreToFirstSample = drive.actionBuilder(AutoPoses.scoreBucketPose)
                .strafeToLinearHeading(spike1Short.position, spike1Short.heading);
        TrajectoryActionBuilder fromFirstSampleToScore = drive.actionBuilder(spike1Short)
                .strafeToLinearHeading(AutoPoses.scoreBucketPose.position, AutoPoses.scoreBucketPose.heading);
        //.turnTo(AutoPoses.scoreBucketPose.heading);

        TrajectoryActionBuilder fromScoreToSecondSample = drive.actionBuilder(AutoPoses.scoreBucketPose)
                .strafeToConstantHeading(spike2Short.position)
                .turnTo(spike2Short.heading);
        //.strafeToLinearHeading(spike2Short.position, spike2Short.heading);
        TrajectoryActionBuilder fromSecondSampleToScore = drive.actionBuilder(spike2Short)
                .strafeToLinearHeading(AutoPoses.scoreBucketPose.position, AutoPoses.scoreBucketPose.heading);

        TrajectoryActionBuilder fromScoreToThirdSample = drive.actionBuilder(AutoPoses.scoreBucketPose)
                .strafeToLinearHeading(spike3Short.position, spike3Short.heading);
        TrajectoryActionBuilder fromThirdSampleToScore = drive.actionBuilder(spike3Short)
                //.strafeToConstantHeading(AutoPoses.shiftPoseByInputs(AutoPoses.thirdSpikeBucketPose, -12, 0, 0).position)
                .strafeToLinearHeading(AutoPoses.scoreBucketPose.position, AutoPoses.scoreBucketPose.heading);

        int timeToGrabSampleMS = 300;

        telemetry.addLine("READY TO START!");
        telemetry.addData("SAMPLE 1", inputtedPose.toString());
        telemetry.addData("\n\nSAMPLE 2", inputtedPose2.toString());
        telemetry.addData("\n\nSAMPLE 3", inputtedPose3.toString());

        telemetry.addData("horizTargetSub", horizTargetSub);
        telemetry.addData("horizTargetSub2", horizTargetSub2);
        telemetry.addData("horizTargetSub3", horizTargetSub3);
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
                                //preload
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(20),
                                autoActions.raiseToHighBucket(),

                                autoActions.waitTillVerticalPastInches(4, true),
                                autoActions.extendHoriz(12),

                                fromStartToScore.build(),
                                autoActions.openGripper(),
                                sleeper.sleep(timeToDropMS),


                                //spike 1
                                new ParallelAction(
                                        fromScoreToFirstSample.build(),
                                        new SequentialAction(
                                                autoActions.waitTillPastY(55, false),
                                                autoActions.lowerToTransfer()
                                        )
                                ),

                                sleeper.sleep(100),

                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(20),

                                sleeper.sleep(timeToGrabSampleMS),
                                autoActions.transfer(),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.extendHoriz(11),
                                autoActions.raiseToHighBucket(),
                                autoActions.waitTillSlidesArePartiallyUp(),
                                fromFirstSampleToScore.build(),
                                autoActions.waitTillSlidesAreAllTheWayUp(),
                                autoActions.openGripper(),

                                sleeper.sleep(timeToDropMS),

                                //spike 2
                                new ParallelAction(
                                        fromScoreToSecondSample.build(),
                                        new SequentialAction(
                                                autoActions.waitTillPastY(55, false),
                                                autoActions.lowerToTransfer()
                                        )
                                ),

                                sleeper.sleep(100),
                                autoActions.setIntakeArmPosition("grab"),
                                sleeper.sleep(timeToGrabSampleMS),
                                autoActions.transfer(),

                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(-30),
                                autoActions.extendHoriz(12),
                                autoActions.raiseToHighBucket(),
                                autoActions.waitTillSlidesArePartiallyUp(),
                                fromSecondSampleToScore.build(),
                                autoActions.waitTillSlidesAreAllTheWayUp(),
                                autoActions.openGripper(),

                                sleeper.sleep(timeToDropMS),

                                //spike 3
                                new ParallelAction(
                                        fromScoreToThirdSample.build(),
                                        new SequentialAction(
                                                autoActions.waitTillPastY(55, false),
                                                autoActions.lowerToTransfer()
                                        )
                                ),

                                sleeper.sleep(100),
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(-30),
                                sleeper.sleep(200),
                                new ParallelAction(
                                        new SequentialAction(
                                                autoActions.extendHoriz(0),
                                                autoActions.waitTillHorizPastInches(10, false),
                                                autoActions.transfer(),
                                                autoActions.raiseToHighBucket()
                                        ),
                                        fromThirdSampleToScore.build()
                                ),

                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.waitTillSlidesAreAllTheWayUp(),
                                autoActions.openGripper(),

                                autoActions.setIntakeArmPosition("limelight"),

                                //SUB INTAKE
                                new ParallelAction(
                                        fromBucketToIntake1.build(),
                                        new SequentialAction(
                                                autoActions.waitTillPastY(50, false),
                                                autoActions.lowerToTransfer(),
                                                autoActions.waitTillPastY(36, false),//16
                                                autoActions.extendHoriz(horizTargetSub),
                                                autoActions.setIntakeRoll(inputtedPose.getRoll())
                                        )
                                ),
                                autoActions.lineUpWithLimelight(inputtedPose, maxHuntingTimeMS),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(200),
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(timeToGrabSampleMS),

                                autoActions.switchLimelightPipeline(inputtedPose2.getColor()),

                                new ParallelAction(
                                        fromIntakeToBucket1.build(),
                                        new SequentialAction(
                                                autoActions.transfer(),
                                                autoActions.raiseToHighBucket()
                                        )
                                ),

                                autoActions.waitTillSlidesAreAllTheWayUp(),

                                autoActions.openGripper(),

                                sleeper.sleep(timeToDropMS),
                                autoActions.setIntakeArmPosition("limelight"),

                                //SUB INTAKE 2
                                new ParallelAction(
                                        fromBucketToIntake2.build(),
                                        new SequentialAction(
                                                autoActions.waitTillPastY(50, false),
                                                autoActions.lowerToTransfer(),
                                                autoActions.waitTillPastY(16, false),
                                                autoActions.extendHoriz(horizTargetSub2),
                                                autoActions.setIntakeRoll(inputtedPose2.getRoll())
                                        )
                                ),
                                autoActions.lineUpWithLimelight(inputtedPose2, maxHuntingTimeMS),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(inputtedPose2.getRoll()),
                                sleeper.sleep(200),
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(inputtedPose2.getRoll()),
                                sleeper.sleep(timeToGrabSampleMS),

                                autoActions.switchLimelightPipeline(inputtedPose3.getColor()),


                                new ParallelAction(
                                        fromIntakeToBucket2.build(),
                                        new SequentialAction(
                                                autoActions.transfer(),
                                                autoActions.raiseToHighBucket()
                                        )
                                ),

                                autoActions.waitTillSlidesAreAllTheWayUp(),

                                autoActions.openGripper(),

                                sleeper.sleep(timeToDropMS),
                                autoActions.setIntakeArmPosition("limelight"),

                                //PARK
                                new ParallelAction(
                                        fromBucketToIntake3.build(),
                                        new SequentialAction(
                                                autoActions.waitTillPastY(50, false),
                                                //autoActions.lowerToPrePark(),
                                                autoActions.raiseToPark(),
                                                autoActions.waitTillPastY(16, false),
                                                autoActions.extendHoriz(horizTargetSub3),
                                                autoActions.setIntakeRoll(inputtedPose3.getRoll())
                                        )
                                ),

                                //autoActions.raiseToPark(),

                                autoActions.lineUpWithLimelight(inputtedPose3, 2000),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(inputtedPose3.getRoll()),
                                sleeper.sleep(300),
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(inputtedPose3.getRoll()),
                                sleeper.sleep(timeToGrabSampleMS),





                                sleeper.sleep(1000),
                                autoActions.endProgram()
                        )
                )
        );


    }
}