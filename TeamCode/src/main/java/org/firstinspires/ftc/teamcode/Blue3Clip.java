package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous
public class Blue3Clip extends LinearOpMode {
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
                if (drive.pose.position.y < 59.9) {
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


        public Action readComponents() {
            return  new ReadComponents();
        }

        public Action writeComponents() {
            return new WriteComponents();
        }


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
        Vector2d grabVector = new Vector2d(-36, 60);//61.5ish for strafe//-32 x sometimes
        double grabAngleRad = Math.toRadians(90);
        Pose2d grabPose = new Pose2d(grabVector, grabAngleRad);

        Vector2d preGrabVector = new Vector2d(grabVector.x, grabVector.y-10);
        double preGrabAngleRad = grabAngleRad;
        Pose2d preGrabPose = new Pose2d(preGrabVector, preGrabAngleRad);
        //Pose2d dropPose = new Pose2d(-60, 48, Math.toRadians(90));

        double grabY = 42;
        Vector2d grabSpark1Vector = new Vector2d(-29.5, grabY+1);
        double grabSpark1Rad = Math.toRadians(225);
        Pose2d grabSpark1Pose= new Pose2d(grabSpark1Vector, grabSpark1Rad);

        Vector2d grabSpark2Vector = new Vector2d(-40.5, grabY);
        double grabSpark2Rad = Math.toRadians(225);
        Pose2d grabSpark2Pose= new Pose2d(grabSpark2Vector, grabSpark2Rad);

        Vector2d grabSpark3Vector = new Vector2d(-45.75, 37.5);
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

        TrajectoryActionBuilder fromScoreToSparkOneToGrab = drive.actionBuilder(scorePose)
                .strafeToConstantHeading(new Vector2d(-12, 48))
                .strafeToConstantHeading(new Vector2d(-36, 48))
                .strafeToConstantHeading(new Vector2d(-36, 12))
                .strafeToConstantHeading(new Vector2d(-48, 12))
                .strafeToConstantHeading(new Vector2d(-48, 52))
                //.strafeToLinearHeading(new Vector2d(preGrabVector.x, preGrabVector.y-10), preGrabAngleRad)
                .strafeToLinearHeading(preGrabVector, preGrabAngleRad)
                .strafeToConstantHeading(grabVector);

        TrajectoryActionBuilder fromGrabToScoreCycle = drive.actionBuilder(grabPose)
                //.splineToConstantHeading(new Vector2d(scoreCycleVector.x-3, grabVector.y-5), Math.toRadians(-45))
                //.splineToConstantHeading(scoreCycleVector, Math.toRadians(270));
                .strafeToConstantHeading(preGrabVector)
                .strafeToConstantHeading(new Vector2d(scoreCycleVector.x, preGrabVector.y))
                .strafeToConstantHeading(scoreCycleVector);


        TrajectoryActionBuilder fromScoreCycleToGrab = drive.actionBuilder(scoreCyclePose)
                //.strafeToConstantHeading(new Vector2d(grabVector.x, grabVector.y-5))
                .strafeToConstantHeading(new Vector2d(scoreCycleVector.x, scoreCycleVector.y+5))
                .strafeToConstantHeading(preGrabVector)
                .strafeToConstantHeading(grabVector);


        TrajectoryActionBuilder fromScoreCycleToPark = drive.actionBuilder(scoreCyclePose)
                .strafeToConstantHeading(new Vector2d(preGrabVector.x-6, preGrabVector.y+6));

        //bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.readHubs();
        bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, -40, 90, 0, false));
        bart.intake.intakeArm.setToSavedIntakeArmPosition("grab");
        //bart.intake.intakeArm.setRollDeg(45);
        //bart.intake.closeGate();
        bart.writeAllComponents();
        //bart.output.sendVerticalSlidesToTarget();

        int timeToGrabSpikeMilliseconds = 320;
        int timeToDropSpikeMilliseconds = 80;

        int timeToGrabClipMilliseconds = 150;
        int timeToDropClipMilliseconds = 25;

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(

                                //SCORE PRELOAD
                                outputs.raiseToHighBarFront(),
                                fromStartToScore.build(),
                                outputs.openGripper(),
                                outputs.lowerToGrab(),

                                //push in the sparkmark
                                fromScoreToSparkOneToGrab.build(),

                                //GRAB CLIP 2 OFF WALL
                                outputs.closeGripper(),
                                sleeper.sleep(timeToGrabClipMilliseconds),

                                //SCORE THE CLIP
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        fromGrabToScoreCycle.build()
                                ),
                                outputs.openGripper(),
                                sleeper.sleep(timeToDropClipMilliseconds),

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
                                outputs.openGripper(),
                                sleeper.sleep(timeToDropClipMilliseconds),

                                outputs.lowerToPark(),
                                fromScoreCycleToPark.build(),


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