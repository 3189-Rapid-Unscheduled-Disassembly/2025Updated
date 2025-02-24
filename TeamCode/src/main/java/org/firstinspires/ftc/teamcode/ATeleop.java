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
        bart.readHubs();

        if (!isIntakeArmJank) {
            bart.output.setTargetToCurrentPosition();
            /*if (bart.output.verticalSlides.currentInches() > 9) {
                bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(17, 0, 0, false));
            } else {
                bart.output.setComponentPositionsFromSavedPosition("straightOut");
            }*/
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


        //HORIZONTAL SLIDE
        //transfer when a is held
        if (playerTwo.isDown(GamepadKeys.Button.A)) {
            if (bucketDrivingMode) {
                bart.transfer();
            } else {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("postGrab");
                bart.intake.setHorizontalSlideToSavedPosition("transfer");
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
                bart.intake.setHorizontalSlideToSavedPosition("transfer");
            //P2 is inputting nothing
            } else if (!playerTwo.isDown(GamepadKeys.Button.A)){
                bart.intake.horizontalSlide.setTargetToCurrentPosition();
                if (bart.intake.horizontalSlide.isAboveMax()) {
                    bart.intake.setHorizontalSlideToSavedPosition("max");
                }
            }
            bart.intake.horizontalSlide.goToTargetAsync();
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
            bart.output.verticalSlides.setPower(0.5 * playerTwo.getLeftY());
            bart.output.setTargetToCurrentPosition();
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


        //HANG CODE
        if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
            bart.output.setComponentPositionsFromSavedPosition("preHang");
            bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
        }
        if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
            bart.output.setComponentPositionsFromSavedPosition("hang");
            bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
        }

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


        //this is kinda jank, wanna find a better way
        //honestly really shocked this even works at all
        /*if (!alternateControl && playerTwo.getRightY() != 0) {
            usingHorizManualControl = false;
        }*/


    }
}


