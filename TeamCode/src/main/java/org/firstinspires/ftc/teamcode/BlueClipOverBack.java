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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueClipOverBack extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Outputs outputs;
    Sleeper sleeper;
    boolean endProgram = false;


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

                //ACTION
                if (!initialized) {
                    bart.output.setComponentPositionsFromSavedPosition("highBarBack");
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

        class RaiseToHighBarBackOnceAwayFromWall implements Action {

            boolean actionIsRunning = true;
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                if (drive.pose.position.y < 59.5) {
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

        class LowerToPark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, -40, 90, 0, false));
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

        class IntakeBackwards implements Action {

            boolean actionIsRunning = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.intake.setIntakeMotorPower(-1);

                return actionIsRunning;
            }

        }
        class IntakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.setIntakeMotorPower(0);
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
                bart.writeAllComponents();
                return !endProgram;
            }
        }

        public Action endProgram() {
            return new EndProgram();
        }

        //public Action lowerOutOfWay() {
            //return new LowerOutOfWay();
        //}

        public Action lowerToGrabOnceXPastNegative24() {
            return new LowerToGrabOnceXPastNegative24();
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

        public Action intakeBackwards(){return new IntakeBackwards();}
        public Action intakeStop(){return new IntakeStop();}

        public Action raiseToHighBarFront() {
            return new RaiseToHighBarFront();
        }
        public Action raiseToHighBarBack() {
            return new RaiseToHighBarBack();
        }
        public Action raiseToHighBarBackOnceAwayFromWall() {
            return new RaiseToHighBarBackOnceAwayFromWall();
        }


        public Action retractHoriz() {
            return new RetractHoriz();
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        //create robot 6 5/16 from wall   x = 32 from wall
        Pose2d beginPose = new Pose2d(-5.5, 63.75, Math.toRadians(90));
        //SCORE POSES
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

        TrajectoryActionBuilder fromScoreToGrab = drive.actionBuilder(scorePose)
                .splineToConstantHeading(new Vector2d(grabVector.x+7, grabVector.y-7), Math.toRadians(90))
                .splineToConstantHeading(grabVector, Math.toRadians(90));

        TrajectoryActionBuilder fromScoreToPark = drive.actionBuilder(scorePose)
                .splineToConstantHeading(new Vector2d(grabVector.x-5, grabVector.y-5), Math.toRadians(135));

        TrajectoryActionBuilder fromGrabToScore = drive.actionBuilder(grabPose)
                .splineToConstantHeading(new Vector2d(grabVector.x, grabVector.y-2), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(scoreVector.x-8, scoreVector.y+12), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(scoreVector.x, scoreVector.y), Math.toRadians(-90));

        TrajectoryActionBuilder fromScoreToPushAndGrab = drive.actionBuilder(scorePose)
                //spike 2
                .splineToConstantHeading(new Vector2d(scoreVector.x, 34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-33, 38), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-39, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-43, 14), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-49, 54), Math.toRadians(90))
                //spike 2
                .splineToConstantHeading(new Vector2d(-49, 36), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-55, 14), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, 54), Math.toRadians(90))
                //spike 3
                /*.splineToConstantHeading(new Vector2d(-58, 48), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-58, 36), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-59, 8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, 60), Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(-60, 52), Math.toRadians(270));*/
                //GRAB FROM WALL
                .splineToConstantHeading(new Vector2d(grabVector.x, grabVector.y-12), Math.toRadians(90))
                .splineToConstantHeading(grabVector, Math.toRadians(90));


        /*TrajectoryActionBuilder fromPushToGrab = drive.actionBuilder(new Pose2d(-60, 56,  Math.toRadians(270)))
                        .splineToConstantHeading(new Vector2d(grabVector.x, grabVector.y-4), Math.toRadians(90))
                        .splineToConstantHeading(grabVector, Math.toRadians(90));*/



        /*TrajectoryActionBuilder fromGrabToScore2 = drive.actionBuilder(grabPose)
                //.splineToLinearHeading(new Pose2d(scoreVector.x+2, 60, Math.toRadians(270)), Math.toRadians(180))
                //.strafeToLinearHeading(new Vector2d(score2Vector.x, 60), Math.toRadians(270))
                //.lineToY(scoreVector.y - drivePastTheBarExtraInches);
                //.strafeToConstantHeading(new Vector2d(score2Vector.x, scoreVector.y - drivePastTheBarExtraInches));
        //.splineToConstantHeading(new Vector2d(scoreVector.x+2, scoreVector.y - drivePastTheBarExtraInches), Math.toRadians(270));
                .splineToConstantHeading(score2Vector, 270);

        TrajectoryActionBuilder fromScore2ToGrab = drive.actionBuilder(score2Pose)
                .strafeToLinearHeading(new Vector2d(grabVector.x, grabVector.y-8), grabAngleRad-Math.toRadians(0.1))
                .strafeToConstantHeading(grabVector);

        TrajectoryActionBuilder fromGrabToScore3 = drive.actionBuilder(grabPose)
                .strafeToLinearHeading(new Vector2d(score3Vector.x, 60), Math.toRadians(270))
                //.lineToY(scoreVector.y - drivePastTheBarExtraInches);
                .strafeToConstantHeading(new Vector2d(score3Vector.x, scoreVector.y - drivePastTheBarExtraInches-2));

        TrajectoryActionBuilder fromScore3ToPark = drive.actionBuilder(score3Pose)
                .strafeToConstantHeading(new Vector2d(grabVector.x, grabVector.y-2));*/





        //bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.readHubs();
        bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, -40, 90, 0, false));
        bart.intake.closeGate();
        bart.writeAllComponents();
        //bart.output.sendVerticalSlidesToTarget();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(

                        outputs.readComponents(),

                        new SequentialAction(
                                //SCORE PRELOAD
                                outputs.raiseToHighBarBack(),
                                fromStartToScore.build(),
                                outputs.openGripper(),

                                //PUSH AND GRAB
                                //outputs.lowerOutOfWay(),
                                outputs.lowerToGrab(),
                                //new ParallelAction(
                                        //fromScoreToPushAndGrab.build(),
                                  //      outputs.lowerToGrabOnceXPastNegative24()
                                //),


                                outputs.intakeBackwards(),
                                fromScoreToPushAndGrab.build(),
                                outputs.intakeStop(),








                                //THE THREE CYCLE THING
                                //fromScoreToGrab.build(),
                                //GRAB CLIP 2 FROM WALL
                                outputs.closeGripper(),
                                sleeper.sleep(500),
                                new ParallelAction(
                                    outputs.raiseToHighBarBackOnceAwayFromWall(),
                                    fromGrabToScore.build()
                                ),
                                outputs.openGripper(),
                                //sleeper.sleep(100),


                                outputs.lowerToGrab(),
                                fromScoreToGrab.build(),
                                //GRAB CLIP 3 FROM WALL
                                outputs.closeGripper(),
                                sleeper.sleep(500),
                                new ParallelAction(
                                    outputs.raiseToHighBarBackOnceAwayFromWall(),
                                    fromGrabToScore.build()
                                ),
                                outputs.openGripper(),
                                sleeper.sleep(100),




                                outputs.lowerToGrab(),
                                fromScoreToGrab.build(),
                                //GRAB CLIP 4 FROM WALL
                                outputs.closeGripper(),
                                sleeper.sleep(500),
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        fromGrabToScore.build()
                                ),
                                outputs.openGripper(),
                                sleeper.sleep(100),


                                //PARK
                                //outputs.lowerToPark(),
                                outputs.lowerToGrab(),
                                fromScoreToPark.build(),





                                outputs.endProgram()
                        ),
                        //SEND COMPONENTS TO POSITION EVERY FRAME
                        outputs.writeComponents()

                )
        );

        //put the horizontal slide back home
        /*while (!bart.intake.isAtSavedPosition("transfer") && opModeIsActive()) {
            bart.intake.setHorizontalSlideToSavedPosition("transfer");
        }
        bart.intake.horizontalSlide.setPower(0);*/


    }
}