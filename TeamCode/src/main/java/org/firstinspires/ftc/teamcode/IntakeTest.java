package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class IntakeTest extends LinearOpMode {

    RobotMain bart;

    double pitchTarget, rollTarget;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bart = new RobotMain(hardwareMap, telemetry);
        bart.intake.intakeArm.setToSavedIntakeArmPosition("straightOut");
        GamepadEx playerOne, playerTwo;

        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);
        waitForStart();

        while(opModeIsActive()) {
            playerOne.readButtons();
            playerTwo.readButtons();

            //bart.intake.setHorizontalSlidePower(-gamepad1.right_stick_y*0.5);
            if (playerTwo.isDown(GamepadKeys.Button.A)) {
                bart.intake.horizontalSlide.setTargetInches(0);
            } else if (playerTwo.isDown(GamepadKeys.Button.B)) {
                bart.intake.horizontalSlide.setTargetInches(14);
            } else {
                bart.intake.horizontalSlide.setPower(-gamepad1.left_stick_y*0.5);
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



            if (playerTwo.wasJustPressed(GamepadKeys.Button.X)) {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("straightOut");
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(0, bart.intake.intakeArm.intakeWristPitch.currentAngleDegrees()+1, 0, false));
            }
            if (playerTwo.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(0, bart.intake.intakeArm.intakeWristPitch.currentAngleDegrees()-1, 0, false));
            }

            //arm
            if (playerOne.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(bart.intake.intakeArm.intakeArm.currentAngleDegrees()+1,
                        bart.intake.intakeArm.intakeWristPitch.currentAngleDegrees(),
                        0, false));
            }
            if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(bart.intake.intakeArm.intakeArm.currentAngleDegrees()-1,
                        bart.intake.intakeArm.intakeWristPitch.currentAngleDegrees(),
                        0, false));
            }
            if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("transfer");
            }
            if (playerOne.wasJustPressed(GamepadKeys.Button.Y)) {
                bart.intake.intakeArm.setToSavedIntakeArmPosition("grab");
            }

            if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
                bart.intake.closeGate();
            }

            if (playerOne.wasJustPressed(GamepadKeys.Button.X)) {
                bart.intake.fullyOpenGate();
            }


            if (playerOne.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bart.intake.partiallyOpenGate();
            }


            bart.writeAllComponents();

            telemetry.addLine(bart.intake.intakeArm.toString());
            telemetry.addLine("\n" + bart.intake.intakeArm.posServoTelemetry());

            telemetry.addData("ticks", bart.intake.horizontalSlide.currentTicks());



            telemetry.update();

        }
    }
}