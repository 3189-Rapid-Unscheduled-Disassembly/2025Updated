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
        bart.intake.intakeArm.setToIntakeArmPosition(new IntakeArmPosition(0,0, false));
        GamepadEx playerOne;

        playerOne = new GamepadEx(gamepad1);
        waitForStart();

        while(opModeIsActive()) {
            playerOne.readButtons();

            //bart.intake.setHorizontalSlidePower(-gamepad1.right_stick_y*0.5);
            if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
                bart.intake.setHorizontalSlideToSavedPosition("transfer");
            } else if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
                bart.intake.setHorizontalSlideToSavedPosition("max");
            } else {
                bart.intake.setHorizontalSlidePower(-gamepad1.left_stick_y*0.5);
            }

            if (playerOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bart.intake.intakeArm.flipFlop();
            }

            //pitchTarget = -playerOne.getRightY()*90;
            //rollTarget = -playerOne.getRightX()*90;//left is positive



            //bart.intake.intakeArm.setIntakeArmPosition(pitchTarget, rollTarget);

            if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bart.intake.intakeArm.moveRollPositive45();
            }
            if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bart.intake.intakeArm.moveRollNegative45();
            }

            if (playerOne.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bart.intake.intakeArm.cycle();
            }


            bart.writeAllComponents();

            telemetry.addLine(bart.intake.intakeArm.toString());
            telemetry.addLine(bart.intake.intakeArm.posServoTelemetry());
            telemetry.update();

        }
    }
}