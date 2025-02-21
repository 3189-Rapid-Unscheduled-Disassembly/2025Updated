package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorOctoQuad;

import java.util.Arrays;

@Autonomous(name = "Blue 0+5")
public class Blue0Plus5 extends LinearOpMode {
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

        int timeToDropClipMilliseconds = 100;
        int timeToGrabClipMilliseconds = 100;
        GamepadEx playerTwo = new GamepadEx(gamepad2);
        int currentlyInputting = 0;//0=x, 1=y, 2=roll
        //double inputtedX = 0;
        //double inputtedY = 12;
        //inputtedRoll = 0;



        double yMax = 14;
        double yMin = 0;

        double xMax = 10;
        double xMin = 0;
        //previous yMax16, yMin
        inputtedPose = new AutoSamplePose(0, 6, 12, 0,
                true, true, true, xMax, xMin, yMax, yMin, 90, -45);

        while (inputtedPose.stillInputting && opModeInInit() && !isStopRequested()) {
            playerTwo.readButtons();

            inputtedPose.inputAutoSamplePose(playerTwo);

            telemetry.addLine(inputtedPose.toString());
            telemetry.addLine("\nPress A to Advance");
            telemetry.update();
        }

        horizTargetSub = xMax-inputtedPose.getX();//2 min



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

        Pose2d intakePose = new Pose2d(23, inputtedPose.getY(), Math.toRadians(180));

        TrajectoryActionBuilder fromScoreToIntake = drive.actionBuilder(AutoPoses.scoreBucketPose)
                .strafeToLinearHeading(AutoPoses.shiftPoseByInputs(intakePose, 20, 0, 0).position, intakePose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                )
                .strafeToConstantHeading(intakePose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );

        TrajectoryActionBuilder fromIntakeToScore = drive.actionBuilder(intakePose)
                .strafeToLinearHeading(AutoPoses.shiftPoseByInputs(intakePose, 20, 0, 0).position, AutoPoses.scoreBucketPose.heading,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                )
                .strafeToConstantHeading(AutoPoses.scoreBucketPose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(AutoPoses.scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(AutoPoses.scoreCycleMinAccel, AutoPoses.scoreCycleMaxAccel)
                );


        //static Pose2d scoreBucketPose = new Pose2d(55.5, 57, Math.toRadians(225));
        Pose2d spike1Short = new Pose2d(54, 53, Math.toRadians(250));//53.5x
        Pose2d spike2Short = new Pose2d(57.25, 54, Math.toRadians(270));
        Pose2d spike3Short = new Pose2d(54, 50, Math.toRadians(300));


        //SCORE POSES
        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(AutoPoses.beginBucketPose)
                .strafeToLinearHeading(AutoPoses.scoreBucketPose.position, AutoPoses.scoreBucketPose.heading);

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

        telemetry.addLine("READY TO START!\n");
        telemetry.addLine(inputtedPose.toString());
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
                                //preload
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(20),
                                autoActions.raiseToHighBucket(),

                                autoActions.waitTillSlidesArePartiallyUp(),

                                autoActions.extendHoriz(14),

                                fromStartToScore.build(),
                                autoActions.openGripper(),
                                sleeper.sleep(100),


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
                                autoActions.extendHoriz(14),
                                autoActions.raiseToHighBucket(),
                                autoActions.waitTillSlidesArePartiallyUp(),
                                fromFirstSampleToScore.build(),
                                autoActions.waitTillSlidesAreAllTheWayUp(),
                                autoActions.openGripper(),

                                sleeper.sleep(100),

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
                                autoActions.extendHoriz(14),
                                autoActions.raiseToHighBucket(),
                                autoActions.waitTillSlidesArePartiallyUp(),
                                fromSecondSampleToScore.build(),
                                autoActions.waitTillSlidesAreAllTheWayUp(),
                                autoActions.openGripper(),

                                sleeper.sleep(100),

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
                                    fromScoreToIntake.build(),
                                    new SequentialAction(
                                        autoActions.waitTillPastY(50, false),
                                        autoActions.lowerToTransfer(),
                                        autoActions.waitTillPastY(16, false),
                                        autoActions.extendHoriz(horizTargetSub),
                                        autoActions.setIntakeRoll(inputtedPose.getRoll())
                                    )
                                ),
                                autoActions.lineUpWithLimelight(inputtedPose),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(200),
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(timeToGrabSampleMS),

                                new ParallelAction(
                                    fromIntakeToScore.build(),
                                    new SequentialAction(
                                        autoActions.transfer(),
                                        autoActions.raiseToHighBucket()
                                    )
                                ),

                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.waitTillSlidesAreAllTheWayUp(),

                                autoActions.openGripper(),

                                sleeper.sleep(100),


                                sleeper.sleep(10000),
                                autoActions.endProgram()
                        )
                )
        );


    }
}