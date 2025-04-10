package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
public class LimelightSampleTracking extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(LimelightTelemetry.class);
    Limelight limelight;
    RobotMain bart;

    GamepadEx playerOne, playerTwo;


    double desiredDistance = 4.3;
    double horizTarget = 8;

    @Override
    public void runOpMode() throws InterruptedException {
        bart = new RobotMain(hardwareMap, telemetry);
        //bart.output.setComponentPositionsFromSavedPosition("straightOut");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("limelight");
        bart.writeAllComponents();

        limelight = new Limelight(hardwareMap,7);

        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            bart.readHubs();

            playerOne.readButtons();
            playerTwo.readButtons();

            if (playerTwo.wasJustPressed(GamepadKeys.Button.A)) {
                bart.intake.intakeArm.setOnlySpecifiedValuesToSavedIntakeArmPosition("preGrab", true, true, false, true);
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.X)) {
                bart.intake.intakeArm.setOnlySpecifiedValuesToSavedIntakeArmPosition("grab", true, true, false, true);
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.B)) {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("limelight");
            }

            if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bart.intake.intakeArm.moveRollPositive45();
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bart.intake.intakeArm.moveRollNegative45();
            }


            if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
                limelight.limelight.captureSnapshot("inputted");
            }


            if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
                if (playerOne.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    limelight.setPipeline(0);
                } else {
                    limelight.setPipeline(7);
                }
            }
            if (playerOne.wasJustPressed(GamepadKeys.Button.X)) {
                if (playerOne.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    limelight.setPipeline(1);
                } else {
                    limelight.setPipeline(8);
                }
            }
            if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
                if (playerOne.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    limelight.setPipeline(2);
                } else {
                    limelight.setPipeline(9);
                }
            }

            //if (limelight.resultExists) {
            //if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            //double distanceError = limelight.getLastResultDistanceInches() - desiredDistance;
            //horizTarget = bart.intake.horizontalSlide.currentInches() + distanceError;
            //}
            //}
            //bart.intake.horizontalSlide.setTargetInches(horizTarget);
            //bart.intake.horizontalSlide.goToTargetAsync();

            limelight.updateLastLimelightResult(4.3);

            double currentInches = bart.intake.horizontalSlide.currentInches();
            if (limelight.resultExists) {
                double distanceError = limelight.getLastResultDistanceInches()-desiredDistance;
                horizTarget = currentInches + distanceError;
            } else {
                horizTarget = currentInches;
            }
           //horizTarget = RobotMath.maxAndMin(horizTarget, currentInches+3, currentInches-3);
            bart.intake.horizontalSlide.setTargetInches(horizTarget);

            bart.intake.horizontalSlide.goToTargetAsync();
            bart.writeAllComponents();


            //telemetry.addLine(limelight.telemetry());
            /*telemetry.addData("pipeline", limelight.getPipeline());
            telemetry.addData("resultExists", limelight.resultExists);
            telemetry.addData("# results", limelight.getDetections().size());
            telemetry.addData("dist", limelight.getLastResultDistanceInches());
            telemetry.addData("ty", limelight.returnLastResultTY());
            telemetry.addData("className", limelight.returnLastClassName());*/

            telemetry.addData("pipeline", limelight.getPipeline());
            //telemetry.addLine(limelight.telemetryOfAll());
            telemetry.addData("BEST", limelight.telemetryLast());

            //telemetry.addData("# of corner", limelight.corners.size());
            //telemetry.addLine(limelight.pointDistance());

            telemetry.addData("\n\nlast ty", limelight.returnLastResultTY());
            telemetry.update();
        }
    }
}
