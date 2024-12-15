package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class ArmMaintenance extends LinearOpMode {
    RobotMain bart;


    @Override
    public void runOpMode() throws InterruptedException {
        bart = new RobotMain(hardwareMap, telemetry);
        waitForStart();
        bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(0, 100, 100, true));
        while(opModeIsActive()) {
            bart.output.sendVerticalSlidesToTarget();
            telemetry.addLine(bart.output.currentPosition().componentValuesIrl());
            telemetry.update();
        }
    }
}
