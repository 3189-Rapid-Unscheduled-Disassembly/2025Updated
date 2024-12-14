package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueClipsNewIntake extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Outputs outputs;
    Sleeper sleeper;
    boolean endProgram = false;

    double horizTarget = 6;
    boolean isTransferingNow = false;


    //OUTPUT SYNCHRONOUS MOVEMENTS ACTIONS IF NEEDED

    class Sleeper {
        ElapsedTime timer = new ElapsedTime();
        class Sleep implements Action {
            boolean initialized = false;
            double sleepTimeMs;

            public Sleep(double sleepTimeMs) {
                this.sleepTimeMs = sleepTimeMs;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }


                return !(timer.milliseconds() > sleepTimeMs);
            }
        }

        public Action sleep(double sleepTimeMs) {
            return new Sleep(sleepTimeMs);
        }
    }
    class Outputs {
        class RaiseToHighBarFront implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                if (!initialized) {
                    bart.output.setComponentPositionsFromSavedPosition("highBarFront");
                    initialized = true;
                }

                //EXIT CONDITIONS
                actionIsRunning = !bart.output.verticalSlides.isAtTarget();
                if (!actionIsRunning) {
                    initialized = false;
                }
                return actionIsRunning;
            }

        }
        class RaiseToHighBarBack implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromSavedPosition("highBarBack");
                return false;
            }

        }

        class RaiseToHighBarBackOnceAwayFromWall implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                if (drive.pose.position.y < 56.9) {
                    bart.output.setComponentPositionsFromSavedPosition("highBarBack");
                    actionIsRunning = false;
                }

                return actionIsRunning;
            }

        }

        class LowerToGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromSavedPosition("grab");
                return false;
            }
        }

        class AboveGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromSavedPosition("aboveGrab");
                return false;
            }
        }


        class LowerOutOfWay implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 0, 0, 112,true));
                return false;
            }
        }



        class LowerToPark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromSavedPosition("rest");
                bart.output.gripper.open();
                return false;
            }
        }

        class OpenGripper implements Action {

            boolean actionIsRunning = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.output.gripper.open();
                return actionIsRunning;
            }

        }

        class CloseGripper implements Action {

            boolean actionIsRunning = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.output.gripper.close();
                return actionIsRunning;
            }

        }

        class RetractHoriz implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                bart.intake.setHorizontalSlideToSavedPosition("transfer");

                //endProgram = !bart.intake.isAtSavedPosition("transfer");


                return true;
            }
        }

        class SendComponentsToPositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.sendVerticalSlidesToTarget();
                return !endProgram;
            }
        }


        class EndProgram implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                endProgram = true;
                return false;
            }
        }

        class LowerToGrabOnceXPastNegative24 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (drive.pose.position.x < -24) {
                    bart.output.setComponentPositionsFromSavedPosition("grab");
                    return false;
                }
                return true;
            }
        }

        class SetIntakeArmPosition implements Action {
            String key;

            public SetIntakeArmPosition(String key) {
                this.key = key;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.intakeArm.setToSavedIntakeArmPosition(key);
                return false;
            }
        }

        class SetIntakeRoll implements Action {
            double roll;

            public SetIntakeRoll(double roll) {
                this.roll = roll;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.intakeArm.setRollDeg(roll);
                return false;
            }
        }

        class SetIntakeGripperClosed implements Action {
            boolean open;

            public SetIntakeGripperClosed(boolean open) {
                this.open = open;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //bart.intake.intakeArm.setGripperPosition(open);
                //bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(-85, 0, false));
                //bart.intake.intakeArm.setToSavedIntakeArmPosition("grab");
                bart.intake.intakeArm.intakeGripper.setPosition(false);
                //bart.intake.intakeArm.setRollDeg(45);
                return false;
            }
        }
        class SetIntakeGripperOpen implements Action {
            boolean open;

            public SetIntakeGripperOpen(boolean open) {
                this.open = open;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //bart.intake.intakeArm.setGripperPosition(open);
                //bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(-85, 0, true));
                //bart.intake.intakeArm.setToSavedIntakeArmPosition("grab");
                //bart.intake.intakeArm.setRollDeg(45);
                bart.intake.intakeArm.intakeGripper.setPosition(true);
                return false;
            }
        }


        class ExtendHoriz implements Action {

            double target = 6;
            public ExtendHoriz(double target) {
                this.target = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                horizTarget = this.target;
                return !bart.intake.isAtPosition(horizTarget);
            }

        }
        class ExtendHorizOnceXLessThanNegative20 implements Action {
            double target = 6;

            public ExtendHorizOnceXLessThanNegative20(double target) {
                this.target = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (drive.pose.position.x < -20) {
                    horizTarget = target;
                    return !bart.intake.isAtPosition(horizTarget);
                }
                return true;
            }
        }
        class ExtendHorizOncePastAngle implements Action {

            double targetInches = 6;
            double targetAngleDeg;
            boolean onceGreaterThan;

            boolean weArePassedTheAngle = false;

            //true=extend once angle larger than inputted angle
            public ExtendHorizOncePastAngle(double targetInches, double targetAngleDeg, boolean onceGreaterThan) {
                this.targetInches = targetInches;
                this.targetAngleDeg = targetAngleDeg;
                this.onceGreaterThan = onceGreaterThan;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!weArePassedTheAngle) {
                    double robotAngleDeg = Math.toDegrees(drive.pose.heading.toDouble());
                    //the .toDouble() returns from -pi to pi rad, so -180 to 180 deg
                    //the math works on a 0 to 360, so we need to change those angles to their equal, but positive
                    //spent too much time trying to find the error
                    if (robotAngleDeg < 0) {
                        robotAngleDeg += 360;
                    }

                    //this side is for if we desire to be larger than the inputted angle
                    if (onceGreaterThan) {
                        if (robotAngleDeg >= targetAngleDeg) {
                            horizTarget = targetInches;
                            weArePassedTheAngle = true;
                        }
                    } else {
                        if (robotAngleDeg <= targetAngleDeg) {
                            horizTarget = targetInches;
                            weArePassedTheAngle = true;
                        }
                    }
                }

                if (weArePassedTheAngle) {
                    return !bart.intake.isAtPosition(horizTarget, 5);
                } else {
                    return true;
                }
            }

        }



        class ReadComponents implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.readHubs();
                return !endProgram;
            }
        }

        class WriteComponents implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isTransferingNow) {
                    bart.intake.setHorizontalSlidePositionInches(horizTarget);
                }
                bart.writeAllComponents();
                return !endProgram;
            }
        }

        public Action endProgram() {
            return new EndProgram();
        }

        public Action lowerOutOfWay() {
            return new LowerOutOfWay();
        }


        public Action lowerToGrabOnceXPastNegative24() {
            return new LowerToGrabOnceXPastNegative24();
        }
        public Action aboveGrab() {
            return new AboveGrab();
        }


        public Action setIntakeGripperClosed(boolean open) {return new SetIntakeGripperClosed(open);}
        public Action setIntakeGripperOpen(boolean open) {return new SetIntakeGripperOpen(open);}

        public Action readComponents() {
            return  new ReadComponents();
        }

        public Action writeComponents() {
            return new WriteComponents();
        }

        public Action setIntakeArmPosition(String key) { return new SetIntakeArmPosition(key);}
        public Action setIntakeRoll(double roll) { return new SetIntakeRoll(roll);}

        public Action sendComponentsToPositions() {
            return new SendComponentsToPositions();
        }

        public Action lowerToGrab() {
            return new LowerToGrab();
        }
        public Action lowerToPark() {
            return new LowerToPark();
        }


        public Action openGripper() {
            return new OpenGripper();
        }

        public Action closeGripper() {
            return new CloseGripper();
        }


        public Action raiseToHighBarFront() {
            return new RaiseToHighBarFront();
        }
        public Action raiseToHighBarBack() {
            return new RaiseToHighBarBack();
        }
        public Action raiseToHighBarBackOnceAwayFromWall() {
            return new RaiseToHighBarBackOnceAwayFromWall();
        }

        public Action extendHoriz(double inches) {return new ExtendHoriz(inches);}
        public Action extendHorizOncePastAngle(double inches, double targetDeg, boolean onceGreaterThan) {return new ExtendHorizOncePastAngle(inches, targetDeg, onceGreaterThan);}
        public Action extendHorizOnceXLessThanNegative20(double inches) {return new ExtendHorizOnceXLessThanNegative20(inches);}
        public Action retractHoriz() {
            return new RetractHoriz();
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        //create robot 6 5/16 from wall   x = 32 from wall
        Pose2d beginPose = new Pose2d(-5.5, 63.75, Math.toRadians(270));
        //SCORE POSES
        Vector2d scoreVector = new Vector2d(beginPose.position.x, 33.5);//36.5 for backwards
        double scoreAngleRad = Math.toRadians(270);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);

        Vector2d scoreCycleVector = new Vector2d(-3, 32);
        double scoreCycleAngleRad = Math.toRadians(90);
        Pose2d scoreCyclePose = new Pose2d(scoreCycleVector, scoreCycleAngleRad);
        //GRAB POSE
        Vector2d grabVector = new Vector2d(-36, 58);//61.5ish for strafe//-32 x sometimes
        double grabAngleRad = Math.toRadians(90);
        Pose2d grabPose = new Pose2d(grabVector, grabAngleRad);
        //Pose2d dropPose = new Pose2d(-60, 48, Math.toRadians(90));

        double grabY = 42;
        Vector2d grabSpark1Vector = new Vector2d(-28, grabY+1);
        double grabSpark1Rad = Math.toRadians(225);
        Pose2d grabSpark1Pose= new Pose2d(grabSpark1Vector, grabSpark1Rad);

        Vector2d grabSpark2Vector = new Vector2d(-40, grabY);
        double grabSpark2Rad = Math.toRadians(225);
        Pose2d grabSpark2Pose= new Pose2d(grabSpark2Vector, grabSpark2Rad);

        Vector2d grabSpark3Vector = new Vector2d(-45, 37.25);
        double grabSpark3Rad = Math.toRadians(190);
        Pose2d grabSpark3Pose= new Pose2d(grabSpark2Vector, grabSpark3Rad);

        double dropRad1 = Math.toRadians(135);
        double dropRad2 = Math.toRadians(120);
        double dropRad3 = Math.toRadians(120);

        /*Vector2d spark2DropVector = new Vector2d(-44.5, 39);
        Pose2d spark2DropPose = new Pose2d(spark2DropVector, dropRad2);*/

        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        outputs = new Outputs();
        sleeper = new Sleeper();

        //horiz target
        //58, 53, 240deg(60deg)
        //58, 48, 270deg




        //DRIVE TRAJECTORIES
        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(beginPose)
                //.lineToY(scoreVector.y - drivePastTheBarExtraInches);
                .splineToConstantHeading(scoreVector, Math.toRadians(270));

        TrajectoryActionBuilder fromScoreToSparkOne = drive.actionBuilder(scorePose)
                .splineToLinearHeading(new Pose2d(scoreVector.x-10, grabSpark1Vector.y, grabSpark1Rad), Math.toRadians(180))
                //.splineToConstantHeading(new Vector2d(-42, 43.5), Math.toRadians(180));//-46
                .strafeToConstantHeading(grabSpark1Vector);
                //.splineToConstantHeading(grabSpark1Vector, Math.toRadians(180));

        TrajectoryActionBuilder fromSparkOneToDrop = drive.actionBuilder(grabSpark1Pose)
                .strafeToLinearHeading(grabSpark2Vector, dropRad1);

        TrajectoryActionBuilder fromDropOneToSparkTwo = drive.actionBuilder(new Pose2d(grabSpark2Vector, dropRad1))
                .turnTo(grabSpark2Rad);


        TrajectoryActionBuilder fromSparkTwoToDrop = drive.actionBuilder(grabSpark2Pose)
                .turnTo(dropRad2);


        TrajectoryActionBuilder fromDropTwoToSparkThree = drive.actionBuilder(new Pose2d(grabSpark2Vector, dropRad2))
                .strafeToLinearHeading(grabSpark3Vector, grabSpark3Rad);

        /*TrajectoryActionBuilder fromSparkThreeToDrop = drive.actionBuilder(grabSpark3Pose)
                .turnTo(dropRad3);

        TrajectoryActionBuilder fromDropThreeToGrab = drive.actionBuilder(new Pose2d(grabSpark3Vector, dropRad3))
                //.strafeToLinearHeading(grabVector, grabAngleRad);
                .splineToLinearHeading(new Pose2d(grabVector.x, grabVector.y-5, grabAngleRad), Math.toRadians(90))
                .splineToConstantHeading(grabVector, Math.toRadians(90));*/

        TrajectoryActionBuilder fromSparkThreeToGrab = drive.actionBuilder(grabSpark3Pose)
                //.splineToLinearHeading(new Pose2d(grabVector.x, grabVector.y-5, grabAngleRad), Math.toRadians(90))
                //.splineToConstantHeading(grabVector, Math.toRadians(90));
                .strafeToLinearHeading(new Vector2d(grabVector.x, grabVector.y-10), grabAngleRad)
                .strafeToConstantHeading(grabVector);



        TrajectoryActionBuilder fromGrabToScoreCycle = drive.actionBuilder(grabPose)
                //.splineToConstantHeading(new Vector2d(scoreCycleVector.x-3, grabVector.y-5), Math.toRadians(-45))
                //.splineToConstantHeading(scoreCycleVector, Math.toRadians(270));
                .strafeToConstantHeading(scoreCycleVector);


        TrajectoryActionBuilder fromScoreCycleToGrab = drive.actionBuilder(scoreCyclePose)
                //.strafeToConstantHeading(new Vector2d(grabVector.x, grabVector.y-5))
                .strafeToConstantHeading(new Vector2d(grabVector.x, grabVector.y-5))
                .strafeToConstantHeading(grabVector);

        TrajectoryActionBuilder fromScoreCycleToPark = drive.actionBuilder(scoreCyclePose)
                .strafeToConstantHeading(new Vector2d(grabVector.x, grabVector.y-4));

        //bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.readHubs();
        bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, -40, 90, 0, false));
        bart.intake.intakeArm.setToSavedIntakeArmPosition("grab");
        //bart.intake.intakeArm.setRollDeg(45);
        //bart.intake.closeGate();
        bart.writeAllComponents();
        //bart.output.sendVerticalSlidesToTarget();

        int timeToGrabSpikeMilliseconds = 350;
        int timeToDropSpikeMilliseconds = 100;

        int timeToGrabClipMilliseconds = 50;

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(

                                //SCORE PRELOAD
                                outputs.setIntakeRoll(45),
                                outputs.raiseToHighBarFront(),
                                fromStartToScore.build(),
                                outputs.openGripper(),

                                //grab the spark marks
                                outputs.lowerOutOfWay(),
                                //grab spark 1
                                new ParallelAction(
                                    fromScoreToSparkOne.build(),
                                    outputs.extendHorizOnceXLessThanNegative20(24)
                                ),
                                outputs.setIntakeGripperClosed(true),
                                sleeper.sleep(timeToGrabSpikeMilliseconds),
                                //drop 1
                                fromSparkOneToDrop.build(),
                                outputs.setIntakeGripperOpen(true),
                                sleeper.sleep(timeToDropSpikeMilliseconds),
                                //grab 2
                                fromDropOneToSparkTwo.build(),
                                outputs.setIntakeGripperClosed(true),
                                sleeper.sleep(timeToGrabSpikeMilliseconds),
                                //drop 2
                                //this is for retracting the horiz to avoid hitting the wall
                                /*new ParallelAction(
                                        fromSparkTwoToDrop.build(),
                                        //determines when to retract and then extend
                                        new SequentialAction(
                                                outputs.extendHorizOncePastAngle(13, 200, false),
                                                outputs.extendHorizOncePastAngle(24, 160, false)
                                        )
                                ),*/
                                fromSparkTwoToDrop.build(),
                                outputs.setIntakeGripperOpen(true),
                                sleeper.sleep(timeToDropSpikeMilliseconds),

                                //grab 3
                                //once again, miss hitting the wall
                                /*new ParallelAction(
                                    fromDropTwoToSparkThree.build(),
                                    new SequentialAction(
                                            outputs.extendHorizOncePastAngle(18, 140, true),//122, 210
                                            outputs.extendHorizOncePastAngle(24, 190, true)
                                    )
                                ),*/
                                fromDropTwoToSparkThree.build(),
                                outputs.setIntakeGripperClosed(true),
                                sleeper.sleep(timeToGrabSpikeMilliseconds+100),

                                //drop 3
                                /*new ParallelAction(
                                        fromSparkThreeToDrop.build(),
                                        new SequentialAction(
                                                outputs.extendHorizOncePastAngle(18, 190, false),
                                                outputs.extendHorizOncePastAngle(18, 140, false)
                                        )
                                ),*/
                                //fromSparkThreeToDrop.build(),
                                //outputs.setIntakeGripperOpen(true),
                                //sleeper.sleep(250),

                                //grab the clip
                                /*outputs.setIntakeArmPosition("preGrab"),
                                outputs.extendHoriz(6),
                                fromDropThreeToGrab.build(),
                                outputs.setIntakeGripperClosed(true),
                                sleeper.sleep(250),
                                //transfer and score
                                new ParallelAction(
                                    fromGrabToScore.build(),

                                    new SequentialAction(
                                        outputs.transferClip(),
                                        outputs.raiseToHighBarBack()
                                    )
                                ),
                                sleeper.sleep(250),*/

                                //DROP 3 AND GRAB CLIP
                                outputs.lowerToGrab(),
                                outputs.setIntakeRoll(45),
                                new ParallelAction(
                                    outputs.extendHoriz(6),
                                    fromSparkThreeToGrab.build()
                                ),
                                outputs.setIntakeGripperOpen(true),
                                sleeper.sleep(50),
                                outputs.closeGripper(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //outputs.aboveGrab(),

                                //SCORE THE CLIP
                                outputs.setIntakeRoll(0),
                                new ParallelAction(
                                    outputs.raiseToHighBarBackOnceAwayFromWall(),
                                    fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 3
                                //grab
                                outputs.lowerToGrab(),
                                fromScoreCycleToGrab.build(),
                                outputs.closeGripper(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 4
                                //grab
                                outputs.lowerToGrab(),
                                fromScoreCycleToGrab.build(),
                                outputs.closeGripper(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 5
                                //grab
                                outputs.lowerToGrab(),
                                fromScoreCycleToGrab.build(),
                                outputs.closeGripper(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        fromGrabToScoreCycle.build()
                                ),

                                outputs.lowerToGrab(),
                                //fromScoreCycleToPark.build(),
                                sleeper.sleep(500),


                                outputs.endProgram()
                        ),
                        //SEND COMPONENTS TO POSITION EVERY FRAME
                        outputs.writeComponents(),
                        outputs.readComponents()
                )
        );

        //put the horizontal slide back home
        /*while (!bart.intake.isAtSavedPosition("transfer") && opModeIsActive()) {
            bart.intake.setHorizontalSlideToSavedPosition("transfer");
        }
        bart.intake.horizontalSlide.setPower(0);*/


    }
}