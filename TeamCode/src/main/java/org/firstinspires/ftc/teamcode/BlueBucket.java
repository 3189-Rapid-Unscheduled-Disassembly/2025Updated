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
                return !bart.output.isAtPosition();
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

        class Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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
                bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 17, 0, true));
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
                    return  !bart.intake.isAtPosition(horizTarget);
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

        public Action extendHoriz(double inches) {return new ExtendHoriz(inches);}
        public Action extendHorizOnceVertOutOfWay(double inches) {return new ExtendHorizOnceVertOutOfWay(inches);}

        public Action retractHoriz() {
            return new RetractHoriz();
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(40, 65.5, Math.toRadians(180));

        //SCORE POSES
        Vector2d scoreVector = new Vector2d(58, 56);
        double scoreAngleRad = Math.toRadians(240);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);

        Vector2d spike1Vector = new Vector2d(49, 46);
        double spike1AngleRad = Math.toRadians(270);
        Pose2d spike1Pose = new Pose2d(spike1Vector, spike1AngleRad);

        Vector2d spike2Vector = new Vector2d(58, 46);
        double spike2AngleRad = Math.toRadians(270);
        Pose2d spike2Pose = new Pose2d(spike2Vector, spike2AngleRad);

        Vector2d spike3Vector = new Vector2d(53, 26.5);
        double spike3AngleRad = Math.toRadians(0);
        Pose2d spike3Pose = new Pose2d(spike3Vector, spike3AngleRad);

        Vector2d parkVector = new Vector2d(21.5, 14);
        double parkAngleRad = Math.toRadians(180);
        Pose2d parkPose = new Pose2d(parkVector, parkAngleRad);


        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        outputs = new Outputs();
        sleeper = new Sleeper();

        //horiz target
        //58, 53, 240deg(60deg)
        //58, 48, 270deg




        //DRIVE TRAJECTORIES
        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToSpikeOne = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(spike1Vector, spike1AngleRad);
        TrajectoryActionBuilder fromSpikeOneToScore = drive.actionBuilder(spike1Pose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToSpikeTwo = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(spike2Vector, spike2AngleRad);
        TrajectoryActionBuilder fromSpikeTwoToScore = drive.actionBuilder(spike2Pose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToSpikeThree = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(spike3Vector, spike3AngleRad);
        TrajectoryActionBuilder fromSpikeThreeToScore = drive.actionBuilder(spike3Pose)
                .strafeToLinearHeading(scoreVector, scoreAngleRad);

        TrajectoryActionBuilder fromScoreToPark = drive.actionBuilder(scorePose)
                .strafeToLinearHeading(new Vector2d(parkVector.x+24, parkVector.y), parkAngleRad)
                .strafeToConstantHeading(parkVector);



        bart.readHubs();
        bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");

        bart.writeAllComponents();

        int timeToDropMilliseconds = 500;
        int timeToGrabMilliseconds = 500;
        int timeAfterTransfer = 100;


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(
                                //SCORE PRELOAD
                                outputs.raiseToHighBucket(),
                                new ParallelAction(
                                        outputs.extendHorizOnceVertOutOfWay(12),
                                        fromStartToScore.build()
                                ),
                                outputs.openGripper(),
                                sleeper.sleep(timeToDropMilliseconds),

                                //SPIKE 1
                                //grab 1
                                new ParallelAction(
                                        fromScoreToSpikeOne.build(),
                                        outputs.lowerToTransferOnceAway()
                                ),
                                outputs.setIntakeArmPosition("grab"),
                                sleeper.sleep(timeToGrabMilliseconds),
                                //transfer 1
                                outputs.transfer(),
                                sleeper.sleep(timeAfterTransfer),
                                outputs.setIntakeArmPosition("preGrab"),
                                outputs.extendHoriz(12),
                                //score 1
                                outputs.raiseToHighBucket(),
                                fromSpikeOneToScore.build(),
                                outputs.openGripper(),
                                sleeper.sleep(timeToDropMilliseconds),

                                //SPIKE 2
                                //grab 2
                                new ParallelAction(
                                        fromScoreToSpikeTwo.build(),
                                        outputs.lowerToTransferOnceAway()
                                ),
                                outputs.setIntakeArmPosition("grab"),
                                sleeper.sleep(timeToGrabMilliseconds),
                                //transfer 2
                                outputs.transfer(),
                                sleeper.sleep(timeAfterTransfer),
                                outputs.setIntakeArmPosition("preGrab"),
                                outputs.extendHoriz(8),
                                outputs.setIntakeRoll(90),
                                //score 2
                                outputs.raiseToHighBucket(),
                                fromSpikeTwoToScore.build(),
                                outputs.openGripper(),
                                sleeper.sleep(timeToDropMilliseconds),

                                //SPIKE 3
                                //grab 3
                                new ParallelAction(
                                        fromScoreToSpikeThree.build(),
                                        outputs.lowerToTransferOnceAway()
                                ),
                                outputs.setIntakeArmPosition("grab"),
                                outputs.setIntakeRoll(90),
                                sleeper.sleep(timeToGrabMilliseconds),
                                //transfer 3
                                outputs.transfer(),
                                outputs.extendHoriz(6),
                                sleeper.sleep(timeAfterTransfer),
                                outputs.setIntakeArmPosition("preGrab"),
                                //score 3
                                outputs.raiseToHighBucket(),
                                fromSpikeThreeToScore.build(),
                                outputs.openGripper(),
                                sleeper.sleep(timeToDropMilliseconds),

                                //PARK
                                outputs.lowerToPrePark(),
                                fromScoreToPark.build(),
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