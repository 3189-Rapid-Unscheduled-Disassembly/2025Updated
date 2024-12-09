package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WristTest extends LinearOpMode {
    Wrist wrist;
    Arm arm;
    Gripper gripper;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        arm.goToAngle(0);

        wrist = new Wrist(hardwareMap);
        wrist.setWristToTarget(0,0, 0);

        gripper = new Gripper(hardwareMap);

        double wristPitchTarget;
        double wristRollTarget;
        waitForStart();
        while (opModeIsActive()) {

            double armAngleTarget = -gamepad2.right_stick_y * 180;
            arm.goToAngle(armAngleTarget);
            wristPitchTarget = -gamepad1.left_stick_y * 90;
            wristRollTarget = gamepad1.right_stick_x * 112;

            wrist.setWristToTarget(wristPitchTarget, wristRollTarget, arm.armAngleCurrent());

            if (gamepad1.right_bumper) {
                gripper.open();
            } else if (gamepad1.left_bumper) {
                gripper.close();
            }

            telemetry.addData("wristLeft", wrist.left.getPosition());
            telemetry.addData("wristRight", wrist.right.getPosition());
            telemetry.addData("wristRobotRel", wristPitchTarget);
            telemetry.addLine(wrist.toString());
            telemetry.addData("armLeft", arm.left.getPosition());

            telemetry.addData("ARM ANGLE", arm.armAngleCurrent());
            telemetry.addLine(gripper.toString());
            telemetry.addData("gripperPos", gripper.front.getPosition());


            telemetry.update();

            wrist.writeServoPositions();
            arm.writeServoPositions();
            gripper.writePosition();
        }

    }
}
