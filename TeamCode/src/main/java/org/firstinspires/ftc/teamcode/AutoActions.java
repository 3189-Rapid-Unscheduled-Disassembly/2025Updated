package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

class AutoActions {
    RobotMain bart;
    MecanumDrive drive;
    Limelight limelight;

    boolean endProgram = false;

    public AutoActions(RobotMain bart, MecanumDrive drive) {
        this.bart = bart;
        this.drive = drive;
    }

    public AutoActions(RobotMain bart, MecanumDrive drive, Limelight limelight) {
        this.bart = bart;
        this.drive = drive;
        this.limelight = limelight;
    }


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


    class SendComponentsToPositions implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            bart.output.sendVerticalSlidesToTarget();
            bart.intake.horizontalSlide.goToTargetAsync();
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
            bart.intake.horizontalSlide.setTargetInches(target);
            return !bart.intake.horizontalSlide.isAtTarget();
        }

    }

    public Action extendHoriz(double target) {return new ExtendHoriz(target);}



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

    class LineUpWithLimelight implements Action {
        boolean isNotLinedUp = true;
        AutoSamplePose inputtedPose;


        public LineUpWithLimelight(AutoSamplePose inputtedPose) {
            this.inputtedPose = inputtedPose;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double desiredDistance;
            //striahgt: 4.3, -6
            //right: 4, -9
            //left: 4, -1
            //thog: 4, -1
            double desiredTy;
            if (inputtedPose.getRoll() == 0) {
                desiredDistance = 4.3;
                desiredTy = -6;
            } else if (inputtedPose.getRoll() == -45) {
                desiredDistance = 4;
                desiredTy = -9;//-5
            } else if (inputtedPose.getRoll() == 45) {
                desiredDistance = 4;
                desiredTy = -1;//5
            } else {
                desiredDistance = 4;
                desiredTy = -1;//7
            }

            double distanceError = limelight.getLastResultDistanceInches()-desiredDistance;
            double horizTarget = bart.intake.horizontalSlide.currentInches()+distanceError;
            bart.intake.horizontalSlide.setTargetInches(horizTarget);

            double errorTy = desiredTy - limelight.returnLastResultTY();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0,
                            -0.28 * Math.signum(errorTy)//0.045
                    ),
                    0
            ));
            drive.updatePoseEstimate();
            if (RobotMath.isAbsDiffWithinRange(limelight.getLastResultDistanceInches(),desiredDistance,0.25) &&
                    RobotMath.isAbsDiffWithinRange(errorTy,0,1.5)) {
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

    public Action lineUpWithLimelight(AutoSamplePose inputtedPose) {return new LineUpWithLimelight(inputtedPose);}

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

    class WaitTillPastY implements Action {
        double targetY;
        boolean onceGreaterThan;


        //true=extend once angle larger than inputted angle
        public WaitTillPastY(double targetY, boolean onceGreaterThan) {
            this.targetY = targetY;
            this.onceGreaterThan = onceGreaterThan;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double currentY = drive.pose.position.y;
            if (onceGreaterThan) {
                return !(currentY >= targetY);
            } else {
                return !(currentY <= targetY);
            }
        }
    }
    public Action waitTillPastY(double targetY, boolean onceGreaterThan) {
        return new WaitTillPastY(targetY, onceGreaterThan);
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
            return !bart.intake.horizontalSlide.isAtPositionInches(0, 2);
        }

    }

    public Action waitTillHorizIsRetracted() {return new WaitTillHorizIsRetracted();}


}

