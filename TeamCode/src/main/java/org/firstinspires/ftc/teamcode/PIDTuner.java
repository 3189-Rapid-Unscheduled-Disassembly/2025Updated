package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class PIDTuner extends LinearOpMode {
    RobotMain bart;
    DcMotorEx motor;
    ElapsedTime timer;
    FtcDashboard ftcDashboard;

    double calculatedPower;
    double error;
    double previousError;
    double summedError;
    double changeInError;
    double previousTime;
    double deltaTime;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static int TARGET_POS = 50;
    public static double INTEGRAL_DECAY = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        bart = new RobotMain(hardwareMap, telemetry);
        timer = new ElapsedTime();
        ftcDashboard = FtcDashboard.getInstance();
        summedError = 0;
        previousTime = 0;

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            bart.readHubs();
            deltaTime = timer.milliseconds() - previousTime;
            previousTime = timer.milliseconds();
            error = TARGET_POS - bart.intake.horizontalSlidePosition;
            summedError += error;
            changeInError = error / deltaTime;
            calculatedPower = (P*error)+(I*summedError)+(D*changeInError);

            bart.intake.setHorizontalSlidePower(calculatedPower);
            summedError -= INTEGRAL_DECAY;
            if (summedError < 0) summedError = 0;
            bart.writeAllComponents();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Current Pos", bart.intake.horizontalSlidePosition);
            packet.put("Target Pos", TARGET_POS);
            packet.put("Error", error);
            ftcDashboard.sendTelemetryPacket(packet);
        }
    }
}
