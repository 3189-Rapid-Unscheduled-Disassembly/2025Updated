package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class LimelightOpencv extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(LimelightTelemetry.class);
    Limelight limelight;
    RobotMain bart;

    GamepadEx playerOne, playerTwo;

    private List<Action> runningActions = new ArrayList<>();

    AutoActions autoActions;

    double desiredDistance = 4.3;
    double horizTarget = 8;

    @Override
    public void runOpMode() throws InterruptedException {
        bart = new RobotMain(hardwareMap, telemetry);
        //bart.output.setComponentPositionsFromSavedPosition("straightOut");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("limelight");
        bart.writeAllComponents();

        limelight = new Limelight(hardwareMap,4);

        autoActions = new AutoActions(bart, bart.mecanaDruve, limelight);


        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);

        double[] desiredAngleArray = new double[1];
        desiredAngleArray[0] = 0;

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            bart.readHubs();

            playerOne.readButtons();
            playerTwo.readButtons();

            if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bart.intake.intakeArm.moveRollPositive45();
                desiredAngleArray[0] = bart.intake.intakeArm.intakeWristRoll.currentAngleDegrees();
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bart.intake.intakeArm.moveRollNegative45();
                desiredAngleArray[0] = bart.intake.intakeArm.intakeWristRoll.currentAngleDegrees();
            }


            //tell the limelight what angle sample we're looking for
            //limelight.limelight.updatePythonInputs(desiredAngleArray);

            //limelight.updateLastLimelightResultEdgeDetection();


            if (playerTwo.wasJustReleased(GamepadKeys.Button.A)) {
                runningActions.clear();
                bart.intake.horizontalSlide.setTargetToCurrentPosition();
                bart.mecanaDruve.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                0
                        ));

            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.A)) {
                //only really care about the pipeline and angle
                AutoSamplePose samplePose = new AutoSamplePose(4, 0, 0, desiredAngleArray[0], true, true, true, 0, 0, 0, 0, 0, 0);
                runningActions.add(
                        new SequentialAction(
                                autoActions.lineUpWithLimelight(samplePose, 1000, desiredAngleArray)
                        )
                );
            }

            if (playerTwo.isDown(GamepadKeys.Button.A)) {
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }

                //check if we need to rebuild the trajectories
                runningActions = newActions;
            }


            if (playerTwo.wasJustPressed(GamepadKeys.Button.B)) {
                limelight.limelight.captureSnapshot("b press");
            }

            bart.writeAllComponents();


            telemetry.addData("pipeline", limelight.getPipeline());
            //telemetry.addLine(limelight.telemetryOfAll());
            telemetry.addData("BEST", limelight.telemetryLast());
            telemetry.addData("angle", desiredAngleArray[0]);


            //telemetry.addData("# of corner", limelight.corners.size());
            //telemetry.addLine(limelight.pointDistance());

            telemetry.addData("\n\nlast ty", limelight.returnLastResultTY());
            telemetry.addData("exists", limelight.resultExists);
            /*for (double val: limelight.outputs) {
                telemetry.addData("", val);
            }*/
            telemetry.update();
        }
    }
}
