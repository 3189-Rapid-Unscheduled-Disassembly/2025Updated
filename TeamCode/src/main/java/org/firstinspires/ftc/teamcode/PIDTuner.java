package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

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

    double loopTime;
    double maxLoopTime;
    double minLoopTime;
    double avgLoopTime;
    double totalTime;
    double totalLoops;


    public static double P = 0.0085;
    public static double I = 0;
    public static double D = 0.1;
    public static int TARGET_POS = 50;
    public static double INTEGRAL_DECAY = 0.1;

    GamepadEx playerOne, playerTwo;



    @Override
    public void runOpMode() throws InterruptedException {
        bart = new RobotMain(hardwareMap, telemetry);
        timer = new ElapsedTime();
        ftcDashboard = FtcDashboard.getInstance();
        summedError = 0;
        previousTime = 0;


        playerOne = new GamepadEx(gamepad1);
        playerTwo = new GamepadEx(gamepad2);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        bart.output.setComponentPositionsFromSavedPosition("straightOut");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("preGrab");

        waitForStart();

        timer.reset();
        while (opModeIsActive()) {
            playerOne.readButtons();
            playerTwo.readButtons();
            bart.readHubs();

            double currentTicks = bart.intake.horizontalSlide.currentTicks();

            deltaTime = timer.milliseconds() - previousTime;
            previousTime = timer.milliseconds();
            error = TARGET_POS - currentTicks;
            summedError += error*deltaTime;
            changeInError = (error-previousError) / deltaTime;
            previousError = error;
            calculatedPower = (P*error)+(I*summedError)+(D*changeInError);

            summedError -= INTEGRAL_DECAY;
            if (summedError < 0) summedError = 0;


            bart.intake.horizontalSlide.setPower(calculatedPower);
            bart.writeAllComponents();




            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Current Pos", currentTicks);
            packet.put("Target Pos", TARGET_POS);
            packet.put("Error", error);
            ftcDashboard.sendTelemetryPacket(packet);



            loopTime = timer.milliseconds() - totalTime;
            if (loopTime > maxLoopTime) maxLoopTime = loopTime;
            if (loopTime < minLoopTime) minLoopTime = loopTime;
            totalLoops++;
            totalTime = timer.milliseconds();
            avgLoopTime = totalTime / totalLoops;
            telemetry.addData("Loop Time", loopTime);
            telemetry.addData("Max Loop Time", maxLoopTime);
            telemetry.addData("Min Loop Time", minLoopTime);
            telemetry.addData("Avg Loop Time", avgLoopTime);
            telemetry.update();
        }
    }
}
