package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class LimelightTelemetry extends LinearOpMode {

    Limelight limelight;
    RobotMain bart;

    GamepadEx playerTwo;

    @Override
    public void runOpMode() throws InterruptedException {
        bart = new RobotMain(hardwareMap, telemetry);
        bart.output.setComponentPositionsFromSavedPosition("straightOut");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("limelight");
        bart.writeAllComponents();

        limelight = new Limelight(hardwareMap,1);

        playerTwo = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
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

            bart.writeAllComponents();

            limelight.updateLastLimelightResult();
            telemetry.addData("dist", limelight.getLastResultDistanceInches());
            telemetry.addData("ty", limelight.returnLastResultTY());
            telemetry.update();
        }
    }
}
