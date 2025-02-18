package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous
public class BlueClipsInput extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Limelight limelight;
    Outputs outputs;
    Sleeper sleeper;
    boolean endProgram = false;

    double horizTarget = 6;
    boolean isTransferingNow = false;

    double horizTargetSub;
    //boolean isBlue = true;
    //double inputtedRoll;
    AutoSamplePose inputtedPose;


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
                if (drive.pose.position.y < 61.5) {//62.1
                    bart.output.setComponentPositionsFromSavedPosition("highBarBack");
                    bart.output.gripper.setPosition(0.22);
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

        class CloseGripperLoose implements Action {

            boolean actionIsRunning = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.output.gripper.setPosition(0.22);
                return actionIsRunning;
            }

        }

        class CloseGripperTightAfterDelay implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //ACTION
                if (drive.pose.position.y < 58) {
                    bart.output.gripper.close();
                    return false;
                }
                return true;
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

        class LineUpWithLimelight implements Action {
            boolean isNotLinedUp = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double desiredDistance = 4;
                //bart.intake.setHorizontalSlidePositionInches(bart.intake.currentInches()+limelight.getLastResultDistanceInches()-4.3);
                horizTarget = bart.intake.currentInches()+limelight.getLastResultDistanceInches()-desiredDistance;

                double desiredTy;
                if (inputtedPose.getRoll() == 0) {
                    desiredTy = 0;
                } else if (inputtedPose.getRoll() == -45) {
                    desiredTy = -4;//-5
                } else if (inputtedPose.getRoll() == 45) {
                    desiredTy = 4;//5
                } else {
                    desiredTy = 6;//7
                }

                double errorTy = desiredTy - limelight.returnLastResultTY();
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                -0.2 * Math.signum(errorTy)//0.045
                        ),
                        0
                ));
                if (RobotMath.isAbsDiffWithinRange(limelight.getLastResultDistanceInches(),desiredDistance,0.25) &&
                    RobotMath.isAbsDiffWithinRange(errorTy,0,3)) {
                    isNotLinedUp = false;
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0,
                                    0
                            ),
                            0
                    ));

                }

                telemetryPacket.put("Result Exists?", limelight.isSeeingResult());
                limelight.updateLastLimelightResult();
                return isNotLinedUp;
            }
        }

        public Action lineUpWithLimelight(){return new LineUpWithLimelight();}

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
        public Action closeGripperLoose() {
            return new CloseGripperLoose();
        }

        public Action closeGripperTightAfterDelay() {
            return new CloseGripperTightAfterDelay();
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


        class WaitTillPastAngle implements Action {
            double targetAngleDeg;
            boolean onceGreaterThan;


            //true=extend once angle larger than inputted angle
            public WaitTillPastAngle(double targetAngleDeg, boolean onceGreaterThan) {
                this.targetAngleDeg = targetAngleDeg;
                this.onceGreaterThan = onceGreaterThan;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double robotAngleDeg = Math.toDegrees(drive.pose.heading.toDouble());
                //the .toDouble() returns from -pi to pi rad, so -180 to 180 deg
                //the math works on a 0 to 360, so we need to change those angles to their equal, but positive
                //spent too much time trying to find the error
                if (robotAngleDeg < 0) {
                    robotAngleDeg += 360;
                }


                if (onceGreaterThan) {
                    return !(robotAngleDeg >= targetAngleDeg);
                } else {
                    return !(robotAngleDeg <= targetAngleDeg);
                }
            }
        }

        public Action waitTillPastAngle(double targetAngle, boolean onceGreaterThan) {
            return new WaitTillPastAngle(targetAngle, onceGreaterThan);
        }
        class CloseGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.closeGate();
                return false;
            }
        }

        class CloseGateOnceXLessThanNegative12 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (drive.pose.position.x < -4) {
                    bart.intake.closeGate();
                    //remove later
                    //bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");
                    return false;
                }
                return true;
            }
        }

        class PartiallyOpenGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.partiallyOpenGate();
                return false;
            }
        }

        class FullyOpenGate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                bart.intake.fullyOpenGate();
                return false;
            }
        }

        class SetGateOncePastAngle implements Action {

            double targetAngleDeg;
            boolean onceGreaterThan;
            int position;


            //true=extend once angle larger than inputted angle
            public SetGateOncePastAngle(double targetAngleDeg, boolean onceGreaterThan, int postion) {
                this.position = postion;
                this.targetAngleDeg = targetAngleDeg;
                this.onceGreaterThan = onceGreaterThan;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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
                        if (position == 0) {
                            bart.intake.closeGate();
                        } else if (position == 1){
                            bart.intake.partiallyOpenGate();
                        } else {
                            bart.intake.fullyOpenGate();
                        }
                        return false;
                    }
                } else {
                    if (robotAngleDeg <= targetAngleDeg) {
                        if (position == 0) {
                            bart.intake.closeGate();
                        } else if (position == 1){
                            bart.intake.partiallyOpenGate();
                        } else {
                            bart.intake.fullyOpenGate();
                        }
                        return false;
                    }
                }

                return true;
            }

        }


        public Action closeGate() {
            return new CloseGate();
        }
        public Action partiallyOpenGate() {
            return new PartiallyOpenGate();
        }
        public Action fullyOpenGate() {
            return new FullyOpenGate();
        }
        public Action closeGateOnceXLessThanNegative12() {
            return new CloseGateOnceXLessThanNegative12();
        }

        public Action setGateOncePastAngle(double targetAngleDeg, boolean onceGreaterThan, int position) {
            return new SetGateOncePastAngle(targetAngleDeg, onceGreaterThan, position);
        }

        class WaitTillHorizIsRetracted implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !bart.intake.isAtSavedPosition("transfer");
            }

        }

        public Action waitTillHorizIsRetracted() {return new WaitTillHorizIsRetracted();}


    }



    @Override
    public void runOpMode() throws InterruptedException {
        //create robot 6 5/16 from wall   x = 32 from wall
        Pose2d beginPose = new Pose2d(-5.5, 63.75, Math.toRadians(270));
        //Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

        double preloadMinAccel = -70;
        double preloadMaxAccel = 70;
        double preloadMaxWheelVel = 70;

        double scoreCycleMinAccel = -75;
        double scoreCycleMaxAccel = 70;
        double scoreCycleGrabMinAccel = -60;
        double scoreCycleGrabMaxAccel = 60;
        double scoreCycleGrabMaxWheelVel = 70;
        double scoreCycleMaxWheelVel = 80;
        double spinnyMaxAngVel = Math.PI * 3.5;


        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        outputs = new Outputs();
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
        //bart.intake.closeGate();
        bart.writeAllComponents();
        //bart.output.sendVerticalSlidesToTarget();

        int timeToDropClipMilliseconds = 100;
        int timeToGrabClipMilliseconds = 50;
        GamepadEx playerTwo = new GamepadEx(gamepad2);
        int currentlyInputting = 0;//0=x, 1=y, 2=roll
        //double inputtedX = 0;
        //double inputtedY = 12;
        //inputtedRoll = 0;

        double yMax = 16;


        inputtedPose = new AutoSamplePose(1, 0, 12, 0,
                false, true, true, 5, -5, 16, 0, 90, -45);

        while (inputtedPose.stillInputting && opModeInInit() && !isStopRequested()) {
            playerTwo.readButtons();

            inputtedPose.inputAutoSamplePose(playerTwo);

            telemetry.addLine(inputtedPose.toString());
            telemetry.addLine("\nPress A to Advance");
            telemetry.update();
        }

        horizTargetSub = yMax-inputtedPose.getY()+6;//2 min



        telemetry.addLine("DONE INPUTTING\nFINAL VALUES");
        //telemetry.addData("COLOR", isBlue ? "BLUE" : "RED");
        /*telemetry.addData("X", i);
        telemetry.addData("Y", inputtedY);
        telemetry.addData("ROLL", inputtedRoll);*/
        //telemetry.addData("horizTargetSub", horizTargetSub);
        telemetry.addLine(inputtedPose.toString());
        telemetry.update();



        //LIMELIGHT STUFF
        if (inputtedPose.getColor() == 1) {
            limelight = new Limelight(hardwareMap, 1);
        } else {
            limelight = new Limelight(hardwareMap, 2);
        }


        //SCORE POSES
        Vector2d scoreVector = new Vector2d(inputtedPose.getX(), 35);
        double scoreAngleRad = Math.toRadians(270);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);

        Pose2d normalizedGrabPose = new Pose2d(inputtedPose.getX(), 29, Math.toRadians(270));

        Pose2d dropSubPose = new Pose2d(-40, 40, Math.toRadians(120));

        Pose2d grabSpark1Pose = new Pose2d(-32, 40, Math.toRadians(240));//-36,36
        Pose2d grabSpark2Pose = new Pose2d(-42, 40, Math.toRadians(240));//-41, 40
        Pose2d grabSpark3Pose = new Pose2d(-46, 26, Math.toRadians(200));


        Pose2d dropSpark1Pose = new Pose2d(grabSpark2Pose.position, Math.toRadians(130));

        Pose2d dropSpark2Pose = new Pose2d(-44, 40, Math.toRadians(130));


        Vector2d scoreCycleVector = new Vector2d(-5, 34);
        double scoreCycleAngleRad = Math.toRadians(90);
        Pose2d scoreCyclePose = new Pose2d(scoreCycleVector, scoreCycleAngleRad);
        //GRAB POSE
        Vector2d grabVector = new Vector2d(-38, 62.5);//61.5ish for strafe//-32 x sometimes
        double grabAngleRad = Math.toRadians(90);
        Pose2d grabPose = new Pose2d(grabVector, grabAngleRad);


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
        TrajectoryActionBuilder fromStartToScore = drive.actionBuilder(beginPose)

            .splineToConstantHeading(scorePose.position, Math.toRadians(270),
                    new MinVelConstraint(Arrays.asList(
                            drive.kinematics.new WheelVelConstraint(preloadMaxWheelVel),
                            new AngularVelConstraint(Math.PI * 1.5)
                    )),
                    new ProfileAccelConstraint(preloadMinAccel, preloadMaxAccel)
            );

        TrajectoryActionBuilder fromScoreToNormalizedGrab = drive.actionBuilder(scorePose)
                .strafeToConstantHeading(normalizedGrabPose.position);

        TrajectoryActionBuilder fromNormalizedGrabToDrop = drive.actionBuilder(normalizedGrabPose)
                .splineToConstantHeading(new Vector2d(normalizedGrabPose.position.x, normalizedGrabPose.position.y + 10), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleMinAccel, scoreCycleMaxAccel)
                )
                .splineToLinearHeading(dropSubPose, Math.toRadians(180),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleMinAccel, scoreCycleMaxAccel)
                );

        TrajectoryActionBuilder fromSubDropToSweep = drive.actionBuilder(dropSubPose)
                //SPIKE 1
                .strafeToLinearHeading(grabSpark1Pose.position, grabSpark1Pose.heading)
                .strafeToLinearHeading(dropSpark1Pose.position, dropSpark1Pose.heading)
                //SPIKE 2
                .turnTo(grabSpark2Pose.heading.toDouble())
                .strafeToLinearHeading(dropSpark2Pose.position, dropSpark2Pose.heading)
                //SPIKE 3
                .splineToLinearHeading(grabSpark3Pose, Math.toRadians(200))
                .splineToConstantHeading(new Vector2d(grabSpark3Pose.position.x, grabSpark3Pose.position.y+6), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(grabSpark3Pose.position.x, 40, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(grabVector.x, grabVector.y-2), Math.toRadians(90));
//                .strafeToLinearHeading(grabSpark3Pose.position, grabSpark3Pose.heading)
//                .strafeToConstantHeading(new Vector2d(grabSpark3Pose.position.x-1, grabSpark3Pose.position.y+10))
//                .strafeToLinearHeading(grabPose.position, grabPose.heading);



        TrajectoryActionBuilder fromScoreToPush = drive.actionBuilder(scorePose)
                //spike 1
                .splineToConstantHeading(new Vector2d(scoreVector.x-3, 35), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20, 35), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-31, 34), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-42, 18), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-48, 45), Math.toRadians(90))
                //spike 2
                .splineToConstantHeading(new Vector2d(-45, 40), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-56, 22), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, 24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-60, 45), Math.toRadians(90))
                //spike 3
                .splineToConstantHeading(new Vector2d(-54, 36), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-60, 22), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-67, 24), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-65.5, 48), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-56, 51), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(grabVector.x-2, grabVector.y-5, Math.toRadians(90)), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(spinnyMaxAngVel)
                        )),
                        new ProfileAccelConstraint(-40, 45))
                .splineToConstantHeading(new Vector2d(grabVector.x, grabVector.y+0.5), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(60),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(-40, 45)
                );

        TrajectoryActionBuilder fromGrabToScoreCycle = drive.actionBuilder(grabPose)
                .strafeToConstantHeading(scoreCycleVector,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleMinAccel, scoreCycleMaxAccel)
                );//26.07 first grab
        //last score: 38.52
        //press play: 11.71

        TrajectoryActionBuilder fromScoreCycleToGrab = drive.actionBuilder(scoreCyclePose)
                .strafeToConstantHeading(grabVector,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleGrabMaxAccel)
                );
                /*
                .splineToConstantHeading(new Vector2d(scoreCycleVector.x, scoreCycleVector.y+2), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleMaxAccel)
                )
                .splineToConstantHeading(new Vector2d(grabVector.x+4, grabVector.y-5), Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleMaxAccel)
                )
                .splineToConstantHeading(grabVector, Math.toRadians(90),
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleMaxAccel)
                );
*/


        telemetry.addLine("READY TO START");
        /*telemetry.addData("X", inputtedX);
        telemetry.addData("Y", inputtedY);
        telemetry.addData("ROLL", inputtedRoll);*/
        telemetry.addLine(inputtedPose.toString());
        telemetry.addData("horizTargetSub", horizTargetSub);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        //SEND COMPONENTS TO POSITION EVERY FRAME
                        outputs.readComponents(),
                        outputs.sendComponentsToPositions(),
                        outputs.writeComponents(),
                        new SequentialAction(

                                //SCORE PRELOAD
                                outputs.raiseToHighBarFront(),
                                //fromStartToScore.build(),
                                outputs.setIntakeArmPosition("limelight"),
                                outputs.setIntakeRoll(inputtedPose.getRoll()),
                                new ParallelAction(
                                        fromStartToScore.build(),
                                        outputs.extendHoriz(horizTargetSub)
                                ),
                                outputs.openGripper(),
                                //sleeper.sleep(100),
                                outputs.lowerOutOfWay(),
                                //sleeper.sleep(100),
                                //grab the sub sample
                                fromScoreToNormalizedGrab.build(),
                                outputs.lineUpWithLimelight(),
                                outputs.setIntakeArmPosition("preGrab"),
                                outputs.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(200),
                                outputs.setIntakeArmPosition("grab"),
                                outputs.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(500),
                                outputs.setIntakeArmPosition("postGrab"),
                                //drop the sub sample
                                new ParallelAction(
                                  fromNormalizedGrabToDrop.build(),
                                  new SequentialAction(
                                          outputs.extendHoriz(6),
                                          outputs.extendHorizOncePastAngle(19, 180, false),
                                          outputs.partiallyOpenGate()
                                  )
                                ),
                                outputs.setIntakeGripperOpen(true),
                                sleeper.sleep(100),
                                outputs.setIntakeArmPosition("rest"),
                                
                                //SWEEP
                                new ParallelAction(
                                    fromSubDropToSweep.build(),
                                    new SequentialAction(
                                            //SPIKE 1
                                            outputs.setGateOncePastAngle(220, true, 0),
                                            //SPIKE 2
                                            outputs.waitTillPastAngle(140, false),
                                            outputs.setGateOncePastAngle(140, true, 1),
                                            outputs.setGateOncePastAngle(220, true, 0),
                                            //SPIKE 3
                                            outputs.waitTillPastAngle(140, false),
                                            outputs.setGateOncePastAngle(140,  true, 1),
                                            outputs.setGateOncePastAngle(180,  true, 0),
                                            outputs.waitTillPastAngle(135, false),
                                            outputs.fullyOpenGate(),
                                            outputs.waitTillPastAngle(100, false),
                                            outputs.extendHoriz(6),

                                            outputs.waitTillHorizIsRetracted(), outputs.lowerToGrab()

                                    )
                                ),






                                //outputs.openGripper(),

                                //grab the spark marks
                                //outputs.lowerOutOfWay(),


                                /*
                                //push the spike marks
                                new ParallelAction(
                                        fromScoreToPush.build(),
                                        outputs.lowerToGrabOnceXPastNegative24()
                                ),

                                //CYCLE
                                //grab clip 1
                                outputs.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //outputs.aboveGrab(),

                                //SCORE THE CLIP
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        outputs.closeGripperTightAfterDelay(),
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
                                outputs.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        outputs.closeGripperTightAfterDelay(),
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
                                outputs.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        outputs.closeGripperTightAfterDelay(),
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
                                outputs.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        outputs.closeGripperTightAfterDelay(),
                                        fromGrabToScoreCycle.build()
                                ),

                                //CYCLE CLIP 6
                                //grab
                                outputs.openGripper(),
                                //outputs.moveWristOutOfWay(),
                                //sleeper.sleep(timeToDropClipMilliseconds),
                                outputs.lowerToGrab(),
                                sleeper.sleep(timeToDropClipMilliseconds),
                                fromScoreCycleToGrab.build(),
                                outputs.closeGripperLoose(),
                                sleeper.sleep(timeToGrabClipMilliseconds),
                                //score
                                new ParallelAction(
                                        outputs.raiseToHighBarBackOnceAwayFromWall(),
                                        outputs.closeGripperTightAfterDelay(),
                                        outputs.lowerIntakeAtEnd(),
                                        fromGrabToScoreCycle.build()
                                ),

                                outputs.openGripper(),

                                //outputs.lowerToGrab(),
                                //sleeper.sleep(timeToDropClipMilliseconds),


                                outputs.moveWristOutOfWay(),
                                //sleeper.sleep(150),

                                //outputs.lowerToPark(),
                                //fromScoreCycleToPark.build(),



                                //3.58, 4.61
                                //1.03, 36 in

                                 */
                                sleeper.sleep(1000),

                                outputs.endProgram()
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