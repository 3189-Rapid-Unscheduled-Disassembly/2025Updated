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
public class BlueBucket extends LinearOpMode {
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

        class RaiseToHighBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromSavedPosition("highBucket");
                //return !bart.output.isAtPosition();
                return false;
            }
        }


        class WaitTillSlidesArePartiallyUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !bart.output.verticalSlides.isAbovePositionInches(10);
            }
        }
        class WaitTillSlidesAreAllTheWayUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !bart.output.verticalSlides.isAbovePositionInches(17.25);
            }
        }

        class LowerToTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromSavedPosition("transfer");
                return false;
            }
        }

        class LowerToTransferOnceAway implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (drive.pose.position.y < 52) {
                    bart.output.setComponentPositionsFromSavedPosition("transfer");
                    return false;
                }
                return true;
            }
        }

        class LowerToPreParkOnceAway implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (drive.pose.position.y < 52) {
                    bart.output.setComponentPositionsFromSavedPosition("straightOut");
                    return false;
                }
                return true;
            }
        }

        class Transfer implements Action {
            boolean intialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!intialized) {
                    bart.firstFrameOfTransfer();
                    intialized = true;
                }
                isTransferingNow = true;
                bart.transfer();
                //we are done
                if (!bart.output.gripper.isOpen() &&
                    bart.intake.intakeArm.isPitchEqualToSavedIntakePosition("preTransfer")
                ) {
                    isTransferingNow = false;
                    return false;
                }
                return true;
            }
        }

        class WaitUntilAwayFromWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return drive.pose.position.x > 43;

            }
        }

        class LowerToPrePark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 0, 0, true));
                return false;
            }
        }

        class RaiseToPark implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 19, 0, true));
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

        class ExtendHorizOnceVertOutOfWay implements Action {

            double target = 6;
            public ExtendHorizOnceVertOutOfWay(double target) {
                this.target = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (bart.output.verticalSlides.currentInches() > 5) {
                    bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");
                    horizTarget = this.target;
                    return !bart.intake.isAtPosition(horizTarget);
                }
                return true;
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

        public Action lowerToTransfer() {
            return new LowerToTransfer();
        }
        public Action lowerToTransferOnceAway() {
            return new LowerToTransferOnceAway();
        }
        public Action lowerToPreParkOnceAway() {return new LowerToPreParkOnceAway();}
        public Action waitUntilAwayFromWall() {return new WaitUntilAwayFromWall();}


        public Action transfer() {
            return new Transfer();
        }

        public Action lowerToPrePark() {
            return new LowerToPrePark();
        }
        public Action raiseToPark() {
            return new RaiseToPark();
        }



        public Action openGripper() {
            return new OpenGripper();
        }

        public Action closeGripper() {
            return new CloseGripper();
        }

        public Action lowerIntakeAtEnd() { return new LowerIntakeAtEnd();}


        public Action raiseToHighBucket() {
            return new RaiseToHighBucket();
        }
        public Action waitTillSlidesArePartiallyUp() {
            return new WaitTillSlidesArePartiallyUp();
        }
        public Action waitTillSlidesAreAllTheWayUp() {return new WaitTillSlidesAreAllTheWayUp();}


        public Action extendHoriz(double inches) {return new ExtendHoriz(inches);}
        public Action extendHorizOnceVertOutOfWay(double inches) {return new ExtendHorizOnceVertOutOfWay(inches);}

        public Action retractHoriz() {
            return new RetractHoriz();
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(40, 66.5, Math.toRadians(180));

        Vector2d scoreVector = new Vector2d(55.5, 57);
        double scoreAngleRad = Math.toRadians(225);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);

        Vector2d firstSampleVector = new Vector2d(48.5, 43);
        double firstSampleAngleRad = Math.toRadians(270);
        Pose2d firstSamplePose = new Pose2d(firstSampleVector, firstSampleAngleRad);

        Vector2d secondSampleVector = new Vector2d(57.5, 44);
        double secondSampleAngleRad = Math.toRadians(270);
        Pose2d secondSamplePose = new Pose2d(secondSampleVector, secondSampleAngleRad);

        Vector2d thirdSampleVector = new Vector2d(54, 27.25);
        double thirdSamplAngleRad = Math.toRadians(360);
        Pose2d thirdSamplePose = new Pose2d(thirdSampleVector, thirdSamplAngleRad);

        Vector2d parkVector = new Vector2d(36, 12);
        double parkAngleRad = Math.toRadians(180);
        Pose2d parkPose = new Pose2d(parkVector, parkAngleRad);

        Vector2d park2Vector = new Vector2d(20, 12);
        double park2AngleRad = Math.toRadians(180);
        Pose2d park2Pose = new Pose2d(park2Vector, park2AngleRad);

        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        outputs = new Outputs();
        sleeper = new Sleeper();

        //horiz target
        //58, 53, 240deg(60deg)
        //58, 48, 270deg



        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToFirstSample = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(firstSampleVector, firstSampleAngleRad);

        TrajectoryActionBuilder fromFirstSampleToScore = drive.actionBuilder(firstSamplePose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToSecondSample = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(secondSampleVector, secondSampleAngleRad);

        TrajectoryActionBuilder fromSecondSampleToScore = drive.actionBuilder(secondSamplePose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToThirdSample = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(thirdSampleVector, thirdSamplAngleRad);

        TrajectoryActionBuilder fromThirdSampleToScore = drive.actionBuilder(thirdSamplePose)
                .strafeToConstantHeading(new Vector2d(thirdSampleVector.x-12, thirdSampleVector.y))
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToPark = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(parkVector, parkAngleRad);

        TrajectoryActionBuilder fromParkToPark2 = drive.actionBuilder(parkPose)
                .strafeToLinearHeading(park2Vector, park2AngleRad);


        bart.readHubs();
        bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");

        bart.writeAllComponents();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(
                                //preload
                                outputs.setIntakeArmPosition("preGrab"),
                                outputs.raiseToHighBucket(),

                                fromStartToScore.build(),
                                outputs.openGripper(),

                                //spike 1
                                new ParallelAction(
                                        fromScoreToFirstSample.build(),
                                        outputs.lowerToTransferOnceAway()
                                ),

                                outputs.setIntakeArmPosition("grab"),

                                sleeper.sleep(1000),
                                outputs.transfer(),
                                outputs.setIntakeArmPosition("preGrab"),
                                outputs.raiseToHighBucket(),
                                fromFirstSampleToScore.build(),
                                outputs.waitTillSlidesAreAllTheWayUp(),
                                outputs.openGripper(),

                                //spike 2
                                new ParallelAction(
                                        fromScoreToSecondSample.build(),
                                        outputs.lowerToTransferOnceAway()
                                ),
                                outputs.setIntakeArmPosition("grab"),
                                sleeper.sleep(1000),
                                outputs.transfer(),
                                outputs.setIntakeArmPosition("preGrab"),
                                outputs.setIntakeRoll(90),
                                outputs.raiseToHighBucket(),
                                fromSecondSampleToScore.build(),
                                outputs.waitTillSlidesAreAllTheWayUp(),
                                outputs.openGripper(),

                                //spike 3
                                new ParallelAction(
                                        fromScoreToThirdSample.build(),
                                        outputs.lowerToTransferOnceAway()
                                ),
                                outputs.setIntakeArmPosition("grab"),
                                outputs.setIntakeRoll(90),
                                sleeper.sleep(1000),
                                new ParallelAction(
                                        new SequentialAction(
                                                outputs.waitUntilAwayFromWall(),
                                                outputs.transfer(),
                                                outputs.raiseToHighBucket()
                                        ),
                                        fromThirdSampleToScore.build()
                                ),

                                outputs.setIntakeArmPosition("preGrab"),
                                outputs.waitTillSlidesAreAllTheWayUp(),
                                outputs.openGripper(),

                                //park
                                new ParallelAction(
                                        fromScoreToPark.build(),
                                        outputs.lowerToPreParkOnceAway()
                                ),
                                fromParkToPark2.build(),
                                outputs.raiseToPark(),

                                sleeper.sleep(10000),
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