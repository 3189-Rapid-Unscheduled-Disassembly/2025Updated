package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class IntakeTest extends LinearOpMode {

    RobotMain bart;

    double pitchTarget, rollTarget;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bart = new RobotMain(hardwareMap, telemetry);
        bart.intake.intakeArm.setToSavedIntakeArmPosition("grab");
        GamepadEx playerTwo;

        playerTwo = new GamepadEx(gamepad2);
        waitForStart();

        while(opModeIsActive()) {
            playerTwo.readButtons();

            //bart.intake.setHorizontalSlidePower(-gamepad1.right_stick_y*0.5);
            if (playerTwo.isDown(GamepadKeys.Button.A)) {
                bart.intake.setHorizontalSlideToSavedPosition("transfer");
            } else if (playerTwo.isDown(GamepadKeys.Button.B)) {
                bart.intake.setHorizontalSlideToSavedPosition("max");
            } else {
                bart.intake.setHorizontalSlidePower(-gamepad1.left_stick_y*0.5);
            }

            if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bart.intake.intakeArm.intakeGripper.flipFlop();
            }


            if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bart.intake.intakeArm.moveRollPositive45();
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bart.intake.intakeArm.moveRollNegative45();
            }

            if (playerTwo.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bart.intake.intakeArm.cycle();
            }


            bart.writeAllComponents();

            if (playerTwo.wasJustPressed(GamepadKeys.Button.X)) {
                bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(0, 0, false));
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(bart.intake.intakeArm.intakePitch.currentAngleDegrees()+1, 0, false));
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(bart.intake.intakeArm.intakePitch.currentAngleDegrees()-1, 0, false));
            }
            telemetry.addLine(bart.intake.intakeArm.toString());
            telemetry.addLine(bart.intake.intakeArm.posServoTelemetry());

            telemetry.addData("ticks", bart.intake.horizontalSlide.getCurrentPosition());

            telemetry.update();

        }
    }
}