package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArmTest extends LinearOpMode {
    RobotMain bart;
    @Override
    public void runOpMode() throws InterruptedException {

        bart = new RobotMain(hardwareMap, telemetry);

        double wristPitchTarget;
        double wristRollTarget;
        double armAngleTarget = 0;
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;

        waitForStart();
        while (opModeIsActive()) {

            //double armAngleTarget = -gamepad2.right_stick_y * 180;

            if (gamepad2.dpad_up && !previousDpadUp) {
                armAngleTarget += 2;
            }

            if (gamepad2.dpad_down && !previousDpadDown) {
                armAngleTarget -= 2;
            }

            previousDpadUp = gamepad2.dpad_up;
            previousDpadDown = gamepad2.dpad_down;

            bart.output.arm.setAngleDegrees(armAngleTarget);
            wristPitchTarget = -gamepad1.left_stick_y * 90;
            wristRollTarget = gamepad1.right_stick_x * 112;

            bart.output.wrist.setAngleDegreesMinusOtherAngle(wristPitchTarget, bart.output.arm.currentAngleDegrees());

            if (gamepad1.right_bumper) {
                bart.output.gripper.open();
            } else if (gamepad1.left_bumper) {
                bart.output.gripper.close();
            }

            telemetry.addLine(bart.output.arm.toString());
            telemetry.addLine(bart.output.arm.toStringServoPos());

            telemetry.addLine(bart.output.wrist.toString());
            telemetry.addLine(bart.output.wrist.toStringServoPos());

            telemetry.addLine(bart.output.gripper.toString());
            telemetry.addLine(bart.output.gripper.toStringServoPos());

            telemetry.update();

            bart.output.writeAllComponents();
        }

    }
}
