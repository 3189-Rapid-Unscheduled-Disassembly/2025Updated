package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.CancelableProfile;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class ATeleopRR extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private RobotMain bart;
    Sleeper sleeper;
    Outputs outputs;
    MecanumDrive drive;


    GamepadEx playerOne, playerTwo;

    Pose2d testStart = new Pose2d(-48, 0, Math.toRadians(270));
    Pose2d testEnd = new Pose2d(-48, -60, Math.toRadians(270));

    TrajectoryActionBuilder test;
    TrajectoryActionBuilder fromGrabToScoreCycle;
    TrajectoryActionBuilder fromScoreCycleToGrab;

    Pose2d scoreCyclePose = new Pose2d(-5, 34, Math.toRadians(90));
    //GRAB POSE
    Vector2d grabVector = new Vector2d(-38, 62.5);//61.5ish for strafe//-32 x sometimes
    double grabAngleRad = Math.toRadians(90);
    Pose2d grabPose = new Pose2d(grabVector, grabAngleRad);

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


    int timeToDropClipMilliseconds = 100;
    int timeToGrabClipMilliseconds = 50;


//    class Trajectories {
//
//        class TestTrajectory implements Action {
//            boolean cancelled = false;
//            MecanumDrive.FollowTrajectoryAction action = new MecanumDrive.FollowTrajectoryAction();
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                test.build();
//
//                return true;
//            }
//        }
//    }

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

        class OpenGripper implements Action {

            boolean actionIsRunning = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                //ACTION
                bart.output.gripper.open();
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

        public Action closeGripperLoose() {
            return new CloseGripperLoose();
        }

        public Action closeGripperTightAfterDelay() {
            return new CloseGripperTightAfterDelay();
        }

        public Action lowerToGrab() {
            return new LowerToGrab();
        }

        public Action raiseToHighBarBackOnceAwayFromWall() {
            return new RaiseToHighBarBackOnceAwayFromWall();
        }

        public Action openGripper() {
            return new OpenGripper();
        }





    }

    @Override
    public void init() {
        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        outputs = new Outputs();
        sleeper = new Sleeper();

        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);

        test = drive.actionBuilder(testStart)
                .strafeToConstantHeading(testEnd.position);

        fromGrabToScoreCycle = drive.actionBuilder(grabPose)
                .strafeToConstantHeading(scoreCyclePose.position,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleMinAccel, scoreCycleMaxAccel)
                );//26.07 first grab

        fromScoreCycleToGrab = drive.actionBuilder(scoreCyclePose)
                .strafeToConstantHeading(grabVector,
                        new MinVelConstraint(Arrays.asList(
                                drive.kinematics.new WheelVelConstraint(scoreCycleGrabMaxWheelVel),
                                new AngularVelConstraint(Math.PI * 1.5)
                        )),
                        new ProfileAccelConstraint(scoreCycleGrabMinAccel, scoreCycleGrabMaxAccel)
                );



        bart.output.setComponentPositionsFromSavedPosition("grab");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");


    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));

        telemetry.update();

        playerOne.readButtons();
        playerTwo.readButtons();
        bart.readHubs();
        //bart.mecanaDruve.updatePoseEstimate();

        // updated based on gamepads
        /*if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
            runningActions.add(new ParallelAction(
                    //new SleepAction(0.5),
                    new InstantAction(() -> bart.intake.closeGate())
            ));
        }*/

        //reset position to start
        //FOLLOW TRAJECTORY
//        if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//            drive.setPosFromOutside(testStart);
//            runningActions.add( new SequentialAction(
//                    test.build()
//            ));
//        }

        if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
            //drive.setPosFromOutside(grabPose);
            runningActions.add(
                    new SequentialAction(
                        outputs.closeGripperLoose(),
                        sleeper.sleep(timeToGrabClipMilliseconds),

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
                        fromScoreCycleToGrab.build()
                    )
            );
        }

        if (playerOne.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            drive.setPosFromOutside(testStart);
        }
//
//        //cancel trajectory if let go
//        //doesn't currently work
//        if (playerOne.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
//            runningActions.add(new InstantAction( () ->
//                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0))
//            ));
//        }

        //MANUAL DRIVE IF NOT PRESSED
//        if (!playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER) && !playerOne.isDown(GamepadKeys.Button.A)) {
//            drive.updatePoseEstimate();
//            runningActions.add(new ParallelAction(
//                    new InstantAction(() ->
//
//            ));
//        }


        //NORMAL TELEOP
        if (!playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 1,
                            -gamepad1.left_stick_x * 1
                    ),
                    -gamepad1.right_stick_x
            ));
        }

        //CLEAR RR ACTIONS
        if (playerOne.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            runningActions.clear();
        }

        //RESET RR POSE
        if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            drive.setPosFromOutside(grabPose);
        }

        if (playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
        }

        //WRITE COMPONENTS
        bart.writeAllComponents();

        //idk where this should go
        //i think just call it last

    }
}

