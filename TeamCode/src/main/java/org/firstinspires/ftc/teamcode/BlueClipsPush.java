package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous
public class BlueClipsPush extends LinearOpMode {
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
                return false;//actionIsRunning;
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
                if (drive.pose.position.y < 62.1) {
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
                bart.output.setComponentPositionsFromSavedPosition("straightOut");
                return false;
            }
        }



        class LowerToPark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 70, 70, true));
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


        class MoveWristOutOfWay implements Action {

            boolean actionIsRunning = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.output.setComponentPositionsFromSavedPosition("highBarBackMoveWrist");
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

        class LowerIntakeAtEnd implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (drive.pose.position.y < 52) {
                    bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");
                    return false;
                }
                return true;
            }
        }

        class RaiseToDropOncePast180 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double robotAngleDeg = Math.toDegrees(drive.pose.heading.toDouble());
                if (robotAngleDeg < 0) {
                    robotAngleDeg += 360;
                }

                if (robotAngleDeg < 180) {
                    bart.intake.intakeArm.setToSavedIntakeArmPosition("drop");
                    return false;
                }
                return true;
            }
        }

        class IntakeGrabOncePast140 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double robotAngleDeg = Math.toDegrees(drive.pose.heading.toDouble());
                if (robotAngleDeg < 0) {
                    robotAngleDeg += 360;
                }

                if (robotAngleDeg > 140) {
                    bart.intake.intakeArm.setOnlySpecifiedValuesToSavedIntakeArmPosition("grab", true, true, false, true);
                    return false;
                }
                return true;
            }
        }


        class SetIntakeRoll implements Action {
            double roll;

            public SetIntakeRoll(double roll) {
                this.roll = roll;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.intakeArm.intakeWristRoll.setAngleDegrees(roll);
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
                if (drive.pose.position.x < -16) {
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

        public Action raiseToDropOncePast180() { return new RaiseToDropOncePast180();}
        public Action intakeGrabOncePast140() { return new IntakeGrabOncePast140();}

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
        public Action moveWristOutOfWay() {
            return new MoveWristOutOfWay();
        }

        public Action closeGripper() {
            return new CloseGripper();
        }

        public Action lowerIntakeAtEnd() { return new LowerIntakeAtEnd();}


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
        //Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

        //SCORE POSES
        Vector2d scoreVector = new Vector2d(beginPose.position.x+2, 33);
        double scoreAngleRad = Math.toRadians(270);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);

        Vector2d scoreCycleVector = new Vector2d(-3, 33);
        double scoreCycleAngleRad = Math.toRadians(90);
        Pose2d scoreCyclePose = new Pose2d(scoreCycleVector, scoreCycleAngleRad);
        //GRAB POSE
        Vector2d grabVector = new Vector2d(-36, 62.25);//61.5ish for strafe//-32 x sometimes
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
                .splineToConstantHeading(scoreVector, Math.toRadians(270),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                );
                //.splineToConstantHeading(new Vector2d(100, 0), Math.toRadians(0));


        TrajectoryActionBuilder fromScoreToPush = drive.actionBuilder(scorePose)
                //spike 1
                .splineToConstantHeading(new Vector2d(scoreVector.x-3, 35), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20, 35), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-31, 34), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-42, 18), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-48, 45), Math.toRadians(90))
                //spike 2
                .splineToConstantHeading(new Vector2d(-48, 40), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-56, 22), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, 24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-60, 45), Math.toRadians(90))
                //spike 3
                .splineToConstantHeading(new Vector2d(-58, 36), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-61, 20), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-62, 24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-62, 52), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-50, 52), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(grabVector.x-2, grabVector.y-5, Math.toRadians(90)), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45))
                .splineToConstantHeading(grabVector, Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                );

        TrajectoryActionBuilder fromGrabToScoreCycle = drive.actionBuilder(grabPose)
                .strafeToConstantHeading(scoreCycleVector,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                );


        TrajectoryActionBuilder fromScoreCycleToGrab = drive.actionBuilder(scoreCyclePose)
                /*.strafeToConstantHeading(grabVector,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                );*/
                .splineToConstantHeading(new Vector2d(scoreCycleVector.x, scoreCycleVector.y+2), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                )
                .splineToConstantHeading(new Vector2d(grabVector.x+4, grabVector.y-5), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                )
                .splineToConstantHeading(grabVector, Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                );

        TrajectoryActionBuilder fromScoreCycleToPark = drive.actionBuilder(scoreCyclePose)
                .strafeToConstantHeading(new Vector2d(grabVector.x, grabVector.y-4));

        //bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.readHubs();
        //bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, -40, 90, false));
        bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
        //bart.intake.intakeArm.setRollDeg(45);
        //bart.intake.closeGate();
        bart.writeAllComponents();
        //bart.output.sendVerticalSlidesToTarget();

        int timeToDropClipMilliseconds = 100;
        int timeToGrabClipMilliseconds = 50;

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(

                                //SCORE PRELOAD
                                outputs.raiseToHighBarFront(),
                                fromStartToScore.build(),
                                outputs.openGripper(),

                                //grab the spark marks
                                outputs.lowerOutOfWay(),

                                //push the spike marks
                                new ParallelAction(
                                    fromScoreToPush.build(),
                                    outputs.lowerToGrabOnceXPastNegative24()
                                ),

                                //CYCLE
                                //grab clip 1
                                outputs.closeGripper(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //outputs.aboveGrab(),

                                //SCORE THE CLIP
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 3
                                outputs.openGripper(),
                                //outputs.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                //grab
                                outputs.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
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
                                outputs.openGripper(),
                                //outputs.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                outputs.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
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
                                outputs.openGripper(),
                                //outputs.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                outputs.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
                                fromScoreCycleToGrab.build(),
                                outputs.closeGripper(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        outputs.lowerIntakeAtEnd(),
                                        fromGrabToScoreCycle.build()
                                ),

                                outputs.openGripper(),

                                //outputs.moveWristOutOfWay(),
                                //sleeper.sleep(150),

                                //outputs.lowerToPark(),
                                //fromScoreCycleToPark.build(),
                                sleeper.sleep(1000),



                                //3.58, 4.61
                                //1.03, 36 in

                                outputs.endProgram()
                        ),
                        //SEND COMPONENTS TO POSITION EVERY FRAME
                        outputs.sendComponentsToPositions(),
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