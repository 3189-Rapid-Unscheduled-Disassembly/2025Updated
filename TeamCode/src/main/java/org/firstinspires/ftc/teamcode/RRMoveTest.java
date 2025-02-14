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
import com.acmerobotics.roadrunner.PoseVelocity2d;
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
public class RRMoveTest extends LinearOpMode {
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

        class StrafeLeft implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.updatePoseEstimate();
                double xError = -60 - drive.pose.position.x;
                double strafeSpeed = xError*-0.05;
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                strafeSpeed
                        ),
                        0
                ));
                if (Math.abs(xError) < 0.5) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0,
                                    strafeSpeed
                            ),
                            0
                    ));
                    return false;
                }
                return true;
            }
        }

        public Action strafeLeft() {return new StrafeLeft(); }

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
        Pose2d beginPose = new Pose2d(-48, 0, Math.toRadians(90));

        Pose2d grabPose = new Pose2d(-48, 24, Math.toRadians(90));

        Pose2d wrong = new Pose2d(-48, 48, Math.toRadians(90));


        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        outputs = new Outputs();
        sleeper = new Sleeper();

        //horiz target
        //58, 53, 240deg(60deg)
        //58, 48, 270deg



        TrajectoryActionBuilder fromStartToGrab = drive.actionBuilder(beginPose)
                        .strafeToConstantHeading(grabPose.position);

        TrajectoryActionBuilder fromGrabToStart = drive.actionBuilder(wrong)
                        .strafeToConstantHeading(beginPose.position);

        bart.readHubs();
        bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");

        bart.writeAllComponents();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        new SequentialAction(
                                fromStartToGrab.build(),
                                //outputs.strafeLeft(),
                                fromGrabToStart.build(),

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