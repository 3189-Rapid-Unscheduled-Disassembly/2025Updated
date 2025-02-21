package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous(name = "BucketSubmersibleTest")
public class BucketSubmersibleTest extends LinearOpMode {
    RobotMain bart;
    MecanumDrive drive;
    Limelight limelight;

    AutoActions autoActions;
    Sleeper sleeper;

    AutoSamplePose inputtedPose;


    double horizTarget;


    //OUTPUT SYNCHRONOUS MOVEMENTS ACTIONS IF NEEDED


    @Override
    public void runOpMode() throws InterruptedException {
        //create robot 6 5/16 from wall   x = 32 from wall
        //Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));



        Pose2d beginPose = new Pose2d(40, 64, Math.toRadians(180));

        bart = new RobotMain(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, beginPose);
        sleeper = new Sleeper();

        //horiz target
        //58, 53, 240deg(60deg)
        //58, 48, 270deg




        Vector2d scoreVector = new Vector2d(55.5, 57);
        double scoreAngleRad = Math.toRadians(225);
        Pose2d scorePose = new Pose2d(scoreVector, scoreAngleRad);


        //bart.output.setComponentPositionsFromSavedPosition("rest");
        bart.readHubs();
        //bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, -40, 90, false));
        bart.output.setComponentPositionsFromSavedPosition("highBarBack");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("rest");
        //bart.intake.intakeArm.setRollDeg(45);
        bart.intake.fullyOpenGate();
        bart.writeAllComponents();
        //bart.output.sendVerticalSlidesToTarget();

        int timeToDropClipMilliseconds = 100;
        int timeToGrabClipMilliseconds = 100;

        autoActions = new AutoActions(bart, drive);


        double yMax = 14;
        double yMin = 0;

        double xMax = 10;
        double xMin = 0;

        inputtedPose = new AutoSamplePose(1, 0, 12, 0,
                true, true, true, xMax, xMin, yMax, yMin, 90, -45);

        GamepadEx playerTwo = new GamepadEx(gamepad2);

        while (inputtedPose.stillInputting && opModeInInit() && !isStopRequested()) {
            playerTwo.readButtons();

            inputtedPose.inputAutoSamplePose(playerTwo);

            telemetry.addLine(inputtedPose.toString());
            telemetry.addLine("\nPress A to Advance");
            telemetry.update();
        }
        horizTarget = xMax-inputtedPose.getX();//2 min

        limelight = new Limelight(hardwareMap, inputtedPose.getColor());


        autoActions = new AutoActions(bart, drive, limelight);



        TrajectoryActionBuilder fromStartToIntake = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(beginPose.position.x, inputtedPose.getY()))
                .strafeToConstantHeading(new Vector2d(23, inputtedPose.getY()));
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

        telemetry.addLine("READY TO START!\n");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(

                new ParallelAction(
                        //SEND COMPONENTS TO POSITION EVERY FRAME
                        autoActions.readComponents(),
                        autoActions.sendComponentsToPositions(),
                        autoActions.writeComponents(),
                        new SequentialAction(
                                fromStartToIntake.build(),
                                autoActions.setIntakeArmPosition("limelight"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                autoActions.extendHoriz(horizTarget),
                                sleeper.sleep(1000),
                                autoActions.lineUpWithLimelight(inputtedPose),
                                autoActions.setIntakeArmPosition("preGrab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(200),//200,150
                                autoActions.setIntakeArmPosition("grab"),
                                autoActions.setIntakeRoll(inputtedPose.getRoll()),
                                sleeper.sleep(300),//500,200
                                autoActions.setIntakeArmPosition("grabCheck"),


                                sleeper.sleep(1000),
                                autoActions.endProgram()
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