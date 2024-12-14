package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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

        Servo gripperServo = hardwareMap.get(Servo.class, "outputGripper");
        gripperServo.setDirection(Servo.Direction.REVERSE);
        gripper = new Gripper(gripperServo, 0.5, 0.1);

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
            telemetry.addLine(gripper.toString());


            telemetry.update();

            wrist.writeServoPositions();
            arm.writeServoPositions();
            gripper.writePosition();
        }

    }
}
