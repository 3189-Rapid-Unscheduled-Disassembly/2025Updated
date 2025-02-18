package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class ATeleop extends LinearOpMode {

    RobotMain bart;


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

    final double xP = 0.05;
    final double yP = 0.05;
    final double thetaP = 0.05;
    final double angularP = 4;
    final double MAX_DRIVE_VELOCITY_MULTIPLIER = 1;
    final double MAX_TURN_VELOCITY_MULTIPLIER = 0.7;


    double p1PreviousRightTrigger = 0;
    double p1PreviousLeftTrigger = 0;

    double p2PreviousRightY = 0;
    double horizTargetInches = 6;
    boolean usingHorizManualControl = false;

    boolean alternateControl = false;
    boolean stupidControl = false;


    //p2 driving mode
    boolean bucketDrivingMode = true;


    double inputtedY = 12;


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

        boolean isIntakeArmJank = false;
        while (!isStarted()) {
            if (gamepad1.a || gamepad2.a) {
                isIntakeArmJank = true;
            }
            telemetry.addData("isIntakeArmJank", isIntakeArmJank);
            telemetry.update();
        }

        waitForStart();
        if (!isIntakeArmJank) {
            bart.output.setComponentPositionsFromSavedPosition("grab");
            bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");
        } else {
            //ONLY USED WHEN INTAKE ARM ENDED JANKILY
            bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 45, 45, true));
            bart.intake.intakeArm.setOnlySpecifiedValuesToSavedIntakeArmPosition("grabCheck", true, false, true, true);
        }


        //visionPortal.stopStreaming();
        cameraIsOn = false;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        while (opModeIsActive()) {

            //READ
            playerOne.readButtons();
            playerTwo.readButtons();
            bart.readHubs();




            //UPDATE
            manualControl();

            //WRITE
            bart.writeAllComponents();
            bart.mecanaDruve.updatePoseEstimate();


            /** TELEMETRY **/
            telemetry.addLine(bucketDrivingMode ? "BUCKET MODE" : "CLIP MODE");
            telemetry.addData("vertInches", bart.output.verticalSlides.currentInches());
            telemetry.addData("vertTicks", bart.output.verticalSlides.currentTicks);
            telemetry.addLine(bart.output.wrist.toString());

            telemetry.addLine(bart.output.wrist.toStringServoPos());

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
            telemetry.addData("Loop Time", loopTime);
            telemetry.addData("Max Loop Time", maxLoopTime);
            telemetry.addData("Min Loop Time", minLoopTime);
            telemetry.addData("Avg Loop Time", avgLoopTime);


            /*telemetry.addData("Angle", Math.toDegrees(bart.mecanaDruve.pose.heading.toDouble()));
            telemetry.addData("p1ly", playerOne.getLeftY());
            telemetry.addData("p1lx", -playerOne.getLeftX());
            telemetry.addData("p1rx", -playerOne.getRightX());*/

            //telemetry.addData("Target Drive Angle", targetAngle);

            /*telemetry.addData("horizInches", bart.intake.currentInches());
            telemetry.addData("horizAmps", bart.intake.horizontalSlide.getCurrent(CurrentUnit.AMPS));
            packet.put("horizAmps", bart.intake.horizontalSlide.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.addData("horizAmps", bart.intake.horizontalSlide.getCurrent(CurrentUnit.AMPS));
            dashboardTelemetry.update();*/
            telemetry.addData("Left X", playerOne.getLeftX());


            telemetry.update();

        }


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
            }
        }
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
                    //bart.intake.setHorizontalSlideToSavedPosition("transfer");
                    //usingHorizManualControl = false;
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
            }

            bart.output.sendVerticalSlidesToTarget();

        }

        if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bart.output.gripper.flipFlop();
        }

        //move intake out of way. time to fight
        if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            bart.intake.intakeArm.setToSavedIntakeArmPosition("fight");
            if (!bucketDrivingMode) {
                bart.output.setComponentPositionsFromSavedPosition("fight");
            }
        }

        //switch drive mode (robot-oriented default, switch to field-oriented)
        /*if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
            bart.switchDriveMode();
        }*/


        //HANG CODE

        if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
            bart.output.setComponentPositionsFromSavedPosition("preHang");
            //if we are already at rest, from doing clips, we can just leave it
            //if (!bart.intake.intakeArm.isPitchEqualToSavedIntakePosition("rest")) {
            //bart.intake.intakeArm.setToSavedIntakeArmPosition("preTransfer");
            //}
            bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
        }
        if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
            bart.output.setComponentPositionsFromSavedPosition("hang");
            //if (!bart.intake.intakeArm.isPitchEqualToSavedIntakePosition("rest")) {
            //bart.intake.intakeArm.setToSavedIntakeArmPosition("preTransfer");
            //}
            bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
        }

        //reset the encoders for the two slide systems
        if (playerTwo.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            //reset vertical and horiz
            if (!alternateControl) {
                bart.resetEncoders();
            } else {
                //reset only the horiz
                bart.intake.resetEncoder();
            }
        }



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
                    if (bart.intake.currentInches() > bart.intake.MAX_POINT - 0.5 && stickInput >= 0) {
                        bart.intake.setHorizontalSlideToSavedPosition("max");
                    } else {
                        bart.intake.setHorizontalSlidePower(stickInput * 0.75);
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
        if (!alternateControl && playerTwo.getRightY() != 0) {
            usingHorizManualControl = false;
        }

        p2PreviousRightY = -playerTwo.getRightY();

    }

    public void checkIfKilled() {
        /*if (!playerOne.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            previousState = currentState;
            currentState = State.MANUAL;
        }*/
    }

    public void doPlayerOnesChecksEachFrame() {
        checkIfKilled();
        inputtedY = RobotMain.dpadInputToChangeValueUpIsNegative(inputtedY, playerOne);
    }
}


