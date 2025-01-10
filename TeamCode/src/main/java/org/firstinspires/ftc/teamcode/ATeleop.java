package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class ATeleop extends LinearOpMode {

    RobotMain bart;


    /**CAMERA**/
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

    final double xP = 0.05;
    final double yP = 0.05;
    final double thetaP = 0.05;
    final double angularP = 4;
    final double MAX_DRIVE_VELOCITY_MULTIPLIER = 0.7;



    double p1PreviousRightTrigger = 0;
    double p1PreviousLeftTrigger = 0;

    double p2PreviousRightY = 0;
    double horizTargetInches = 6;
    boolean usingHorizManualControl = false;

    boolean alternateControl = false;

    //p2 driving mode
    boolean bucketDrivingMode = true;


    double inputtedY = 12;

    enum State {
        MANUAL,
        GO_TO_INPUTTED_Y,
        LINE_UP_INTAKE_UNTIL_USER_CONFIRMS,
        INTAKE,
        DRIVE_TO_BUCKET,
        RELOCALIZE
    }

    State currentState;
    State previousState;

    ElapsedTime elapsedTime;

    double loopTime;
    double maxLoopTime;
    double minLoopTime;
    double avgLoopTime;
    double totalTime;
    double totalLoops;

    double targetAngle;
    boolean usingAutoAngle = false;

    boolean isTransferring = false;


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
        bart = new RobotMain(hardwareMap, telemetry);

        //Create Gamepads
        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);

        //Bulk Cache Read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        bart.output.setComponentPositionsFromSavedPosition("grab");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");

        currentState = State.MANUAL;
        previousState = currentState;

        //visionPortal.stopStreaming();
        cameraIsOn = false;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (opModeIsActive()) {

            //READ
            playerOne.readButtons();
            playerTwo.readButtons();
            bart.readHubs();
            bart.mecanaDruve.updatePoseEstimate();

            //UPDATE
            stateMachine();

            //WRITE
            bart.writeAllComponents();

            /** TELEMETRY **/
            telemetry.addLine(bucketDrivingMode ? "BUCKET MODE" : "CLIP MODE");
            telemetry.addData("vertInches", bart.output.verticalSlides.currentInches());
            telemetry.addData("vertTicks", bart.output.verticalSlides.currentTicks);

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
            avgLoopTime = totalTime/totalLoops;
            /*telemetry.addData("Loop Time", loopTime);
            telemetry.addData("Max Loop Time", maxLoopTime);
            telemetry.addData("Min Loop Time", minLoopTime);
            telemetry.addData("Avg Loop Time", avgLoopTime);*/


            /*telemetry.addData("Angle", Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()));
            telemetry.addData("p1ly", playerOne.getLeftY());
            telemetry.addData("p1lx", -playerOne.getLeftX());
            telemetry.addData("p1rx", -playerOne.getRightX());*/

            //telemetry.addData("Target Drive Angle", targetAngle);

            telemetry.addData("horizInches", bart.intake.currentInches());


            telemetry.update();

        }



    }



    public void stateMachine() {
        switch (currentState) {
            case MANUAL:
                //LOGIC
                manualControl();


                //EVENTS
                /*if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    thogLockToNextCounterClockwise();
                }
                if (playerOne.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    thogLockToNextClockwise();
                }*/
                if (playerOne.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    bart.output.gripper.flipFlop();
                }


                //RESET THE GYRO
                //USE THE DPAD DIRECTION CORRESPONDING TO ROBOT'S DIRECTION
                if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bart.resetGyro(0);
                    targetAngle = 0;
                }
                if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bart.resetGyro(90);
                    targetAngle = 90;
                }
                if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bart.resetGyro(-90);
                    targetAngle = -90;
                }
                if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bart.resetGyro(180);
                    targetAngle = 180;
                }

                break;

            case GO_TO_INPUTTED_Y:
                //LOGIC
                pControllerToPos(new Pose2d(30, inputtedY, Math.toRadians(180)));

                //EVENTS
                if (atPos(new Pose2d(30, inputtedY, Math.toRadians(180)))) {
                    currentState = State.LINE_UP_INTAKE_UNTIL_USER_CONFIRMS;
                }
                checkIfKilled();
                break;

            case LINE_UP_INTAKE_UNTIL_USER_CONFIRMS:
                //LOGIC
                bart.driveRobotRelative(0, playerOne.getLeftX(), 0);

                //EVENTS
                if(playerOne.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    currentState = State.INTAKE;
                }
                checkIfKilled();
                break;

            case INTAKE:
                //LOGIC
                bart.driveFieldRelative(getXSpeed(20), playerOne.getLeftX(), 0);

                //once we get there do the intaking until we get one
                //do it later
                //for now just press b to move on
                if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
                    currentState = State.DRIVE_TO_BUCKET;
                }

                //when switching to DRIVE_TO_BUCKET switch on visionPortal and set cameraOn to true

                checkIfKilled();

                break;

            case DRIVE_TO_BUCKET:
                //LOGIC
                if (bart.mecanaDruve.pose.position.x < 30) {
                    bart.driveFieldRelative(0.5, 0, 0);
                } else {
                    //pControllerToPos(
                    //        new Pose2d(RobotMain.BUCKET_POINT.getPointBlue().toVector2d(),
                    //        RobotMain.BUCKET_POINT.getPointBlue().angleBetweenPoints(bart.robotPosAsPoint2d())));
                    //Localize with April Tags
                    //bart.mecanaDruve.setPosFromOutside(localize());
                }


                //rasie the amr and stuff to high bucket
                //be able to swtich to low bucket if we fill it all up
                //and we wanna

                //do the relocalizing here

                //EVENTS
                if (atPos(new Pose2d(RobotMain.BUCKET_POINT.getPointBlue().toVector2d(),
                        RobotMain.BUCKET_POINT.getPointBlue().angleBetweenPoints(bart.robotPosAsPoint2d())))) {
                    //open gripper
                    currentState = State.GO_TO_INPUTTED_Y;
                    //visionPortal.stopStreaming();
                    cameraIsOn = false;

                }
                checkIfKilled();
                break;

            case RELOCALIZE:
                //just doing that, nothgin else
                break;

                //do later alan thign if we do
        }
    }


    public void thogLockToNextClockwise() {
        usingAutoAngle = true;
        if (targetAngle > 90) {
            targetAngle = 90;
        } else if (targetAngle > 0) {
            targetAngle = 0;
        } else if (targetAngle > -90) {
            targetAngle = -90;
        } else if (targetAngle > -180) {
            targetAngle = -180;
        } else {
            targetAngle = 90;
        }
    }

    public void thogLockToNextCounterClockwise() {
        usingAutoAngle = true;
        if (targetAngle < -90) {
            targetAngle = -90;
        } else if (targetAngle < 0) {
            targetAngle = 0;
        } else if (targetAngle < 90) {
            targetAngle = 90;
        } else if (targetAngle < 180) {
            targetAngle = 180;
        } else {
            targetAngle = -90;
        }
    }

    public double goToTargetAngle() {
        double angleError = targetAngle - Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble());
        if (angleError > 180) {
            angleError = angleError - 360;
        } else if (angleError < -180) {
            angleError = angleError + 360;
        }
        return angularP * (angleError / 180);
    }


    public void pControllerToPos(Pose2d desiredPos) {
        Pose2d difference = desiredPos.minusExp(bart.mecanaDruve.pose);
        //desiredPos.minusExp()
        //double xDiff = desiredPos.position.x - bart.mecanaDruve.pose.position.x;
        //double yDiff = desiredPos.position.y - bart.mecanaDruve.pose.position.y;
        //double thetaDiff = desiredPos.heading.toDouble() -


            bart.driveFieldRelative(
                    difference.position.x*xP,
                    difference.position.y*yP,
                    difference.heading.toDouble()*thetaP);

    }

    public double getXSpeed(double desiredX) {
        return  (desiredX-bart.mecanaDruve.pose.position.x)*xP;
    }

    public boolean atPos(Pose2d desiredPos) {
        return  RobotMath.isAbsDiffWithinRange(desiredPos.position.x, bart.mecanaDruve.pose.position.x, 0.5) &&
                RobotMath.isAbsDiffWithinRange(desiredPos.position.y, bart.mecanaDruve.pose.position.y, 0.5) &&
                RobotMath.isAbsDiffWithinRange(desiredPos.heading.toDouble(), bart.mecanaDruve.pose.heading.toDouble(), Math.toRadians(5));
    }


    public void manualControl() {

        if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
            isFieldRelative = !isFieldRelative;
        }
        if (!isFieldRelative) {
            //bart.driveRobotRelative(playerOne.getLeftY(), playerOne.getLeftX(), playerOne.getRightX());
            goToTargetAngle();
            //if (playerOne.getRightX() != 0) targetAngle -= gamepad1.right_stick_x*5;
            double turnSpeed;
            if (playerOne.getRightX() != 0) {
                turnSpeed = -playerOne.getRightX() * MAX_DRIVE_VELOCITY_MULTIPLIER;
                targetAngle = Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble());
                usingAutoAngle = false;
            } else {
                if (usingAutoAngle) {
                    turnSpeed = goToTargetAngle();
                } else {
                    turnSpeed = 0;
                }

            }
            bart.mecanaDruve.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*MAX_DRIVE_VELOCITY_MULTIPLIER,
                            -gamepad1.left_stick_x*MAX_DRIVE_VELOCITY_MULTIPLIER
                    ),
                    turnSpeed
            ));
        } else {
            bart.driveFieldRelative(playerOne.getLeftY()*MAX_DRIVE_VELOCITY_MULTIPLIER,
                    -playerOne.getLeftX()*MAX_DRIVE_VELOCITY_MULTIPLIER,
                    -playerOne.getRightX()*MAX_DRIVE_VELOCITY_MULTIPLIER);
        }



        alternateControl = playerTwo.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8;

        //transfer when a is held, the left trigger tells it to do a clip transfer
        if (playerTwo.isDown(GamepadKeys.Button.A)) { //|| playerOne.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            if (bucketDrivingMode) {
                bart.transfer();
            } else {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("postGrab");
                bart.intake.setHorizontalSlideToSavedPosition("transfer");
                usingHorizManualControl = false;
            }
            isTransferring = true;
        } else {
            isTransferring = false;
        }
            //bart.intake.setHorizontalSlideToSavedPosition("transfer");
            //usingHorizManualControl = false;
            //bart.intake.intakeArm.setToSavedIntakeArmPosition("transfer");
            //bart.output.setComponentPositionsFromSavedPosition("transfer");


        //INTAKE CONTROL
        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bart.intake.intakeArm.cycle();
        }
        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bart.intake.intakeArm.moveRollPositive45();
        }
        if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bart.intake.intakeArm.moveRollNegative45();
        }

        if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            bart.intake.intakeArm.intakeGripper.flipFlop();
        }

        //OUTPUT CONTROL
        //manual output control
        if (playerTwo.getLeftY() != 0) {
            bart.output.verticalSlides.setSlidePower(0.5 * playerTwo.getLeftY());
            bart.output.setTargetToCurrentPosition();
        } else {
            //BUTTON CONTROL OF THE OUTPUT


            /*if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bart.output.setComponentPositionsFromSavedPosition("grab");
            }*/
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
                //we can use the left trigger to get an alternate position
                if (!alternateControl) {
                    bart.output.setComponentPositionsFromSavedPosition("highBarFront");
                } else {
                    bart.output.setComponentPositionsFromSavedPosition("highBarBack");
                }
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.X)) {
                if (bucketDrivingMode) {
                    bart.output.setComponentPositionsFromSavedPosition("transfer");
                } else {
                    if (!alternateControl) {
                        bart.output.setComponentPositionsFromSavedPosition("grab");
                    } else {
                        bart.output.setComponentPositionsFromSavedPosition("aboveGrab");
                    }
                }
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
               bucketDrivingMode = !bucketDrivingMode;
            }

            bart.output.sendVerticalSlidesToTarget();

        }

        if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bart.output.gripper.flipFlop();
        }

        //switch drive mode (robot-oriented default, switch to field-oriented)
        if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
            bart.switchDriveMode();
        }


        //toggle camera
        if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
            if (cameraIsOn) {
                //visionPortal.stopStreaming();
            } else {
                //visionPortal.resumeStreaming();
            }
        }

        //reset the encoders for the two slide systems
        if (playerTwo.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bart.resetEncoders();
        }


        //stupid trigger control
        //right trigger forward
        if (playerOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1 && p1PreviousRightTrigger != 1) {
            if (!bart.intake.isAtPosition(17, 7)) {
                horizTargetInches = 17;
                usingHorizManualControl = true;
                bart.intake.intakeArm.setToSavedIntakeArmPosition("transfer");
            } else {
                 if (bart.intake.intakeArm.isPitchEqualToSavedIntakePosition("transfer")) {
                    bart.intake.intakeArm.setToSavedIntakeArmPosition("grab");
                } else {
                    if (bart.intake.intakeArm.intakeGripper.isOpen()) {
                        bart.intake.intakeArm.intakeGripper.close();
                    } else {
                        horizTargetInches = 6;
                        usingHorizManualControl = true;
                        bart.intake.intakeArm.setToSavedIntakeArmPosition("transfer");
                    }
                }
            }
        }
        //left trigger backward
        if (playerOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1 && p1PreviousLeftTrigger != 1) {
            if (bart.intake.isAtPosition(17, 7)) {
                if (bart.intake.intakeArm.isPitchEqualToSavedIntakePosition("transfer")) {
                    horizTargetInches = 6;
                    usingHorizManualControl = true;
                } else {
                    if (!bart.intake.intakeArm.intakeGripper.isOpen()) {
                        bart.intake.intakeArm.intakeGripper.open();
                    } else {
                        bart.intake.intakeArm.setToSavedIntakeArmPosition("transfer");
                    }
                }
            } else {
                horizTargetInches = 17;
                usingHorizManualControl = true;
                bart.intake.intakeArm.setToSavedIntakeArmPosition("transfer");
            }
        }

        p1PreviousRightTrigger = playerOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        p1PreviousLeftTrigger = playerOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);



        //horizontal slide
        if (playerTwo.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            bart.intake.setHorizontalSlideToSavedPosition("transfer");
            usingHorizManualControl = false;
        } else if (playerTwo.isDown(GamepadKeys.Button.A) || playerOne.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            //bart.intake.setHorizontalSlideToSavedPosition("transfer");
            usingHorizManualControl = false;
            //bart.intake.intakeArm.setToSavedIntakeArmPosition("transfer");
            //bart.output.setComponentPositionsFromSavedPosition("transfer");
        } else if (!isTransferring) {
            double stickInput = -playerTwo.getRightY();

            if (alternateControl) {
                //this is where i can flick the stick forward to move an inch at a time
                //easier for fine control
                if (stickInput == 1 && p2PreviousRightY != 1) {
                    usingHorizManualControl = true;
                    horizTargetInches = bart.intake.currentInches() + 1;
                } else if (stickInput == -1 && p2PreviousRightY != -1) {
                    usingHorizManualControl = true;
                    horizTargetInches = bart.intake.currentInches() - 1;
                }
            }

            if (usingHorizManualControl) {
                bart.intake.setHorizontalSlidePositionInches(horizTargetInches);
            } else {
                //stick control
                if (!alternateControl) {
                    if (bart.intake.currentInches() > bart.intake.MAX_POINT-0.5 && stickInput > 0) {
                        bart.intake.setHorizontalSlideToSavedPosition("max");
                    } else {
                        bart.intake.setHorizontalSlidePower(stickInput*0.75);
                    }
                    if (playerTwo.getRightY() != 0) {
                        usingHorizManualControl = false;
                    }
                }
            }

        } else {
            usingHorizManualControl = false;
        }

        //this is kinda jank, wanna find a better way
        //honestly really shocked this even works at all
        if (!alternateControl &&  playerTwo.getRightY() != 0) {
            usingHorizManualControl = false;
        }

        p2PreviousRightY = -playerTwo.getRightY();

    }

    public void checkIfKilled() {
        if (!playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            previousState = currentState;
            currentState = State.MANUAL;
        }
    }
    public void doPlayerOnesChecksEachFrame() {
        checkIfKilled();
        inputtedY = RobotMain.dpadInputToChangeValueUpIsNegative(inputtedY, playerOne);
    }

    /**
     * Initialize the AprilTag processor.
     */
    /*private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
*/
/*
    private Pose2d localize() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                detection.robotPose.getOrientation();
                return new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw());
            }
        }
        return bart.mecanaDruve.pose;
    }*/
}


