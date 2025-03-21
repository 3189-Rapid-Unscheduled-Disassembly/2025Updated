package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@TeleOp
public class ATeleop extends LinearOpMode {

    RobotMain bart;


    boolean isTransferring = false;
    boolean hasSentComponentsToBucket = false;

    boolean isp1ltDownThisFrame = false;
    boolean wasp1ltDownLastFrame = false;
    boolean wasp1ltJustPressed = false;



    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    AutoActions autoActions;
    Sleeper sleeper;

    TrajectoryActionBuilder fromGrabToScoreCycle;
    TrajectoryActionBuilder fromScoreCycleToGrab;


    /**
     * CAMERA
     **/
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private boolean cameraIsOn;
    boolean isFieldRelative;

    final double MAX_DRIVE_VELOCITY_MULTIPLIER = 1;

    boolean alternateControl = false;
    boolean stupidControl = false;


    //p2 driving mode
    boolean bucketDrivingMode = true;

    ElapsedTime elapsedTime;

    double loopTime;
    double maxLoopTime;
    double minLoopTime;
    double avgLoopTime;
    double totalTime;
    double totalLoops;

    double targetAngle;
    boolean usingAutoAngle = false;

    GamepadEx playerOne, playerTwo;

    @Override
    public void runOpMode() throws InterruptedException {
        //Camera
        //initAprilTag();
        loopTime = 0;
        maxLoopTime = 0;
        minLoopTime = 99999;
        avgLoopTime = 0;
        totalTime = 0;
        totalLoops = 0;


        targetAngle = 0;

        isFieldRelative = false;


        //create robot
        bart = new RobotMain(hardwareMap, telemetry, false);

        fromGrabToScoreCycle = AutoPoses.fromGrabToScoreCycleTeleop(bart.mecanaDruve);
        fromScoreCycleToGrab = AutoPoses.fromScoreCycleToGrabTeleop(bart.mecanaDruve);


        //Create Gamepads
        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);

        //Bulk Cache Read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        autoActions = new AutoActions(bart, bart.mecanaDruve);
        sleeper = new Sleeper();


        boolean isIntakeArmJank = false;
        boolean isOutputArmJank = false;
        while (!isStarted()) {
            if (gamepad1.a || gamepad2.a) {
                isIntakeArmJank = true;
            }
            if (gamepad1.b || gamepad2.b) {
                isOutputArmJank = true;
            }
            telemetry.addData("isIntakeArmJank", isIntakeArmJank);
            telemetry.addData("isOutputArmJank", isOutputArmJank);
            telemetry.update();
        }

        waitForStart();
        bart.readHubs();

        if (!isOutputArmJank) {
            bart.output.setComponentPositionsFromSavedPosition("grab");
//            if (bart.output.verticalSlides.isAbovePositionInches(2)) {
//                bart.output.setTargetToCurrentPosition();
//            }
        } else {
            bart.output.setTargetToCurrentPosition();
        }

        if (!isIntakeArmJank) {
            bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");
            //bart.intake.intakeArm.intakeGripper.close();
        } else {
            bart.intake.intakeArm.intakeGripper.close();
        }


        //visionPortal.stopStreaming();
        cameraIsOn = false;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();



        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();


            //READ
            playerOne.readButtons();
            playerTwo.readButtons();
            bart.readHubs();


            //allow trigger presses, basiclly one frame
            isp1ltDownThisFrame = playerOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8;

            wasp1ltJustPressed = isp1ltDownThisFrame && !wasp1ltDownLastFrame;

            wasp1ltDownLastFrame = isp1ltDownThisFrame;


            if (playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                autoControl(packet);
            } else {
                manualControl();
            }

            if (playerOne.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
                runningActions.clear();
            }
            if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bart.mecanaDruve.setPosFromOutside(AutoPoses.grabWallClipsPose);
            }


            //WRITE
            bart.hooks.goToTarget();
            bart.writeAllComponents();
            if (!playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                bart.mecanaDruve.updatePoseEstimate();
            }


            /** TELEMETRY **/
            telemetry.addLine(bucketDrivingMode ? "BUCKET MODE" : "CLIP MODE");

            telemetry.addData("hooksTicks", bart.hooks.currentTicks());
            telemetry.addData("hooksTarget", bart.hooks.currentTarget());
            telemetry.addData("hooksIsAtTarget", bart.hooks.isAtTarget());
            telemetry.addData("vertInches", bart.output.verticalSlides.currentInches());
            telemetry.addData("horizInches", bart.intake.horizontalSlide.currentInches());
            telemetry.addData("horizTicks", bart.intake.horizontalSlide.currentTicks());
            telemetry.addData("hoirzIsAboveMax", bart.intake.horizontalSlide.isAboveMax());

            //telemetry.addData("transferVertInches", bart.output.savedPositions.get("transfer").slideInches);
            //telemetry.addData("transferHorizInches", bart.intake.savedPositions.get("transfer"));


            //telemetry.addLine(bart.output.wrist.toString());

            //telemetry.addLine(bart.output.wrist.toStringServoPos());



            //telemetry.addLine(bart.output.currentPosition().pointTelemetry());
            //telemetry.addData("\n", bart.output.currentPosition().componentValuesIrl());
            //telemetry.addLine(bart.output.wrist.toString());
            //telemetry.addData("wristServosLeft", bart.output.wrist.getCurrentLeftServoPosition());
            //telemetry.addData("wristServosRight", bart.output.wrist.getCurrentRightServoPosition());
            //telemetry.addData("armServos", bart.output.arm.getCurrentServoPosition());


            //LOOP TIMER THINGY. MAYBE WANT LATER
            loopTime = elapsedTime.milliseconds() - totalTime;
            if (loopTime > maxLoopTime) maxLoopTime = loopTime;
            if (loopTime < minLoopTime) minLoopTime = loopTime;
            totalLoops++;
            totalTime = elapsedTime.milliseconds();
            avgLoopTime = totalTime / totalLoops;
            /*telemetry.addData("Loop Time", loopTime);
            telemetry.addData("Max Loop Time", maxLoopTime);
            telemetry.addData("Min Loop Time", minLoopTime);
            telemetry.addData("Avg Loop Time", avgLoopTime);*/


            /*telemetry.addData("Angle", Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()));
            telemetry.addData("p1ly", playerOne.getLeftY());
            telemetry.addData("p1lx", -playerOne.getLeftX());
            telemetry.addData("p1rx", -playerOne.getRightX());*/

            //telemetry.addData("Target Drive Angle", targetAngle);

            //telemetry.addData("horizInches", bart.intake.currentInches());
            //telemetry.addData("horizAmps", bart.intake.horizontalSlide.getCurrent(CurrentUnit.AMPS));
            packet.put("horizAmps", bart.intake.horizontalSlide.motors.get(0).getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("horizAmps", bart.intake.horizontalSlide.motors.get(0).getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.update();
//            telemetry.addData("Left X", playerOne.getLeftX());


            telemetry.update();

        }


    }


    public void autoControl(TelemetryPacket packet) {
        if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
            //drive.setPosFromOutside(grabPose);
            runningActions.add(
                    new SequentialAction(
                            autoActions.closeGripperLoose(),
                            sleeper.sleep(AutoPoses.timeToGrabClipMilliseconds),

                            //SCORE THE CLIP
                            new ParallelAction(
                                    autoActions.raiseToHighBarBackOnceAwayFromWall(),
                                    autoActions.closeGripperTightAfterDelay(),
                                    fromGrabToScoreCycle.build()
                            ),

                            //CYCLE CLIP 3
                            autoActions.openGripper(),
                            //outputs.moveWristOutOfWay(),
                            //sleeper.sleep(timeToDropClipMilliseconds),
                            //grab
                            autoActions.lowerToGrab(),
                            sleeper.sleep(AutoPoses.timeToDropClipMilliseconds),
                            fromScoreCycleToGrab.build()
                    )
            );
        }

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
    public void manualControl() {

        double turnSpeed;
        //if (playerOne.getRightX() != 0) targetAngle -= gamepad1.right_stick_x*5;

        double p1rsx = -playerOne.getRightX() * 1;//MAX_TURN_VELOCITY_MULTIPLIER;
        boolean turnSpeedIsPositive = p1rsx >= 0;
        p1rsx = Math.abs(p1rsx);
        if (p1rsx < 0.025) {
            turnSpeed = 0;
        } else if (p1rsx < 0.3) {
            turnSpeed = 0.15;
        } else {
            turnSpeed = p1rsx * 0.7;
        }
        if (!turnSpeedIsPositive) {
            turnSpeed = -turnSpeed;
        }
        targetAngle = Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble());

        bart.mecanaDruve.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        playerOne.getLeftY() * MAX_DRIVE_VELOCITY_MULTIPLIER,
                        -playerOne.getLeftX() * MAX_DRIVE_VELOCITY_MULTIPLIER
                ),
                turnSpeed
        ));


        alternateControl = playerTwo.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8;
        stupidControl = playerTwo.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.8;


        //RESET THE TRANSFER TIMER ONLY WHEN A IS FIRST PRESSED
        if (playerTwo.wasJustPressed(GamepadKeys.Button.A)) {
            if (bucketDrivingMode) {
                bart.firstFrameOfTransfer();
                isTransferring = true;
                hasSentComponentsToBucket = false;
            }
        }


        //transfer when a is held, then go to high bucket once done transferring
        //idk why but maybe we need to do the press thing for some reason
        if (playerTwo.isDown(GamepadKeys.Button.A) && !playerTwo.wasJustPressed(GamepadKeys.Button.A)) {
            //idk maybe we need to do this
            if (bucketDrivingMode) {
                //transfer, and then flag it as done once it is completed
                if (isTransferring) {
                    bart.transfer();
                    if (bart.isTransferDone()) {
                        isTransferring = false;
                    }
                    hasSentComponentsToBucket = false;
                } else {
                    //we are done transferring, so we can raise to bucket
                    //if p2 holds down the right trigger, then we will delay the bucket
                    //we can also change the target with other inputs
                    //we also don't want to set them each frame, so that way we can manually control, for stuff like drop
                    if (!hasSentComponentsToBucket) {
                        if (!stupidControl) {
                            if (alternateControl) {
                                bart.output.setComponentPositionsFromSavedPosition("lowBucket");
                            } else if (playerOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.8) {
                                bart.output.setComponentPositionsFromSavedPosition("highBucketFlat");
                            } else {
                                bart.output.setComponentPositionsFromSavedPosition("highBucket");
                            }
                            //we just sent the slides, so we don't need to send them each frame
                            //or else we couldn't manually change slide position or open the grabber
                            hasSentComponentsToBucket = true;
                            bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");
                        }
                    } else {
                        if (alternateControl) {
                            bart.output.setComponentPositionsFromSavedPosition("lowBucket");
                        } else if (playerOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.8) {
                            //keep the slides in the same spot
                            double currentSlideInches = bart.output.verticalSlides.currentInches();
                            bart.output.setComponentPositionsFromSavedPosition("highBucketFlat");
                            bart.output.verticalSlides.setTargetInches(currentSlideInches);
                        } else if (wasp1ltJustPressed) {
                            if (bart.output.verticalSlides.isAbovePositionInches(bart.output.savedPositions.get("highBucket").slideInches-1)) {
                                bart.output.verticalSlides.setTargetInches(bart.output.verticalSlides.currentInches()+2);
                            } else {
                                bart.output.verticalSlides.setTargetInches(bart.output.savedPositions.get("highBucket").slideInches+2);
                            }
                        }
                    }
                }
            } else {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("postGrab");
                bart.intake.setHorizontalSlideToSavedPosition("transfer");
            }
        }

        //on the let go of A, we want to bring everything back down to the transfer position
        //this will allow for easier control of the output. (only needs one button for p2)
        // and p2 can instantly send the slides back if we have a bad transfer
        if (playerTwo.wasJustReleased(GamepadKeys.Button.A)) {
            if (bucketDrivingMode) {
                bart.output.setComponentPositionsFromSavedPosition("transfer");
                isTransferring = false;
                //hasSentComponentsToBucket = false;
            }
        }

        //not transfer control
        if (playerTwo.getRightY() != 0) {
            //up on right y is negative, for some reason
            if (bart.intake.horizontalSlide.isAboveMax() && playerTwo.getRightY() < 0) {
                bart.intake.setHorizontalSlideToSavedPosition("max");
                bart.intake.horizontalSlide.goToTargetAsync();
            } else {
                bart.intake.horizontalSlide.setPower(-playerTwo.getRightY() * 0.75);
                bart.intake.horizontalSlide.setTargetToCurrentPosition();
            }
        } else {
            if (playerTwo.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                if (!alternateControl) {
                    if (!stupidControl) {
                        bart.intake.setHorizontalSlideToSavedPosition("transfer");
                    }
                }
            //P2 is inputting nothing
            } else if (!isTransferring){
                bart.intake.horizontalSlide.setTargetToCurrentPosition();
                if (bart.intake.horizontalSlide.isAboveMax()) {
                    bart.intake.setHorizontalSlideToSavedPosition("max");
                }
            }

            //if (!playerTwo.isDown(GamepadKeys.Button.A)) {
            bart.intake.horizontalSlide.goToTargetAsync();
            //}
        }


        //INTAKE CONTROL
        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if (alternateControl) {
                bart.intake.changeSavedPositionByInches("transfer", 0.25);
            } else if (stupidControl) {
                //move the transfer pose up quickly
                Objects.requireNonNull(bart.output.savedPositions.get("transfer")).changeSlideTargetByInches(0.25);
            } else {
                bart.intake.intakeArm.cycle();
            }
        }

        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bart.intake.intakeArm.moveRollPositive45();
        }
        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bart.intake.intakeArm.moveRollNegative45();
        }

        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            if (alternateControl) {
                bart.intake.changeSavedPositionByInches("transfer", -0.25);
            } else if (stupidControl) {
                Objects.requireNonNull(bart.output.savedPositions.get("transfer")).changeSlideTargetByInches(-0.25);
            }
        }

        if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            bart.intake.intakeArm.intakeGripper.flipFlop();
        }

        //OUTPUT CONTROL
        //manual output control
        if (playerTwo.getLeftY() != 0) {
            bart.output.verticalSlides.setPower(0.5 * playerTwo.getLeftY());
            bart.output.setTargetToCurrentPosition();
        } else if (playerOne.isDown(GamepadKeys.Button.A)) {
            //bart.output.setComponentPositionsFromSavedPosition("hangLevel3");
            bart.output.level3Hang();
            if (!bart.output.verticalSlides.isAbovePositionInches(10)) {
                bart.hooks.depower();
            }
        } else {
            //BUTTON CONTROL OF THE OUTPUT
            if (playerTwo.wasJustPressed(GamepadKeys.Button.Y)) {
                if (bucketDrivingMode) {
                    //we can use the left trigger to get an alternate position
                    if (!alternateControl) {
                        bart.output.setComponentPositionsFromSavedPosition("highBucket");
                    } else {
                        bart.output.setComponentPositionsFromSavedPosition("lowBucket");
                    }
                    //lower the intake arm back down to get ready for the next cycle
                    bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");
                } else {
                    bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
                    bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 45, 45, true));
                }
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.B)) {
                //score the low park
                if (bucketDrivingMode) {
                    bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 20, 0, true));
                } else {
                    //we can use the left trigger to get an alternate position
                    if (!stupidControl) {
                        if (!alternateControl) {
                            bart.output.setComponentPositionsFromSavedPosition("highBarFront");
                        } else {
                            bart.output.setComponentPositionsFromSavedPosition("highBarBack");
                        }
                    } else {
                        bart.intake.intakeArm.setToSavedIntakeArmPosition("lowBar");
                        bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 20, 0, true));
                    }
                }
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.X)) {
                if (bucketDrivingMode) {
                    if (!alternateControl) {
                        bart.output.setComponentPositionsFromSavedPosition("transfer");
                    } else {
                        //use this to reset vertical slides better
                        bart.output.setComponentPositionsFromSavedPosition("grab");
                    }
                } else {
                    if (!stupidControl) {
                        if (!alternateControl) {
                            bart.output.setComponentPositionsFromSavedPosition("grab");
                        } else {
                            bart.output.setComponentPositionsFromSavedPosition("aboveGrab");
                        }
                    } else {
                        bart.intake.intakeArm.setToSavedIntakeArmPosition("stupidGrab");
                        bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 20, 0, true));
                    }
                }
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bucketDrivingMode = !bucketDrivingMode;
                //if we switch to clips, we wanna raise the arm so that way we can do near
                if (!bucketDrivingMode) {
                    bart.output.setComponentPositionsFromSavedPosition("straightOut");
                }

                //don't think this is really necessary, but it should remove any weird left over stuff
                isTransferring = false;
                hasSentComponentsToBucket = false;
            }

            //ADJUST HIGH BUCKET SCORING
            /*if (playerOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8) {
                if (bucketDrivingMode) {
                    if (bart.output.verticalSlides.isAbovePositionInches(10)) {
                        bart.output.setComponentPositionsFromSavedPosition("highBucketVertical");
                    }
                }
            }*/
            if (bucketDrivingMode) {
                if (playerOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.8) {
                    if (bucketDrivingMode) {
                        if (bart.output.verticalSlides.isAbovePositionInches(10)) {
                            //bart.output.setComponentPositionsFromSavedPosition("highBucketFlat");
                        }
                    }
                }
            }


            bart.output.sendVerticalSlidesToTarget();
        }

        if (playerOne.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bart.output.gripper.flipFlop();
        }
        if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bart.output.gripper.flipFlop();
        }

        if (playerOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8) {
            if (!bucketDrivingMode) {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("fight");
                bart.output.setComponentPositionsFromSavedPosition("fight");
            } else {
                if (!playerTwo.isDown(GamepadKeys.Button.A)) {
                    bart.intake.intakeArm.setToSavedIntakeArmPosition("fight");
                }
            }
        }



        //move intake out of way. time to fight
        if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            /*bart.intake.intakeArm.setToSavedIntakeArmPosition("fight");
            if (!bucketDrivingMode) {
                bart.output.setComponentPositionsFromSavedPosition("fight");
            }*/
        }


        //HANG CODE
        if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
            bart.output.setComponentPositionsFromSavedPosition("preLevel2");
            bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
            bart.hooks.setTargetToPreHang();
        }
        if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
            //bart.output.setComponentPositionsFromSavedPosition("hang");
            //bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
            bart.hooks.setTargetToHang();
        }

        if (playerOne.isDown(GamepadKeys.Button.X)) {
            if (!bart.output.verticalSlides.isAbovePositionInches(20)) {
                bart.output.setComponentPositionsFromSavedPosition("preHangArmBackLevel3");
            } else {
                bart.output.setComponentPositionsFromSavedPosition("preHangLevel3");
            }
        }

        if (playerOne.wasJustReleased(GamepadKeys.Button.A)) {
            bart.output.verticalSlides.setTargetToCurrentPosition();
        }
        /*if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
            bart.hooks.setTarget(0);
        }*/

        //reset the encoders for the two slide systems
        if (playerTwo.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            //reset vertical and horiz
            if (!alternateControl) {
                bart.resetEncoders();
            } else {
                //reset only the horiz
                bart.intake.horizontalSlide.resetEncoder();
            }
        }
    }
}


