package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hooks {

    DcMotorEx motor;

    private boolean depowered = false;

    private double currentTicks = -3600;
    private double targetTicks = -3600;

    private double currentPower = -3600;
    private double previousPower = -3600;

    final double POWER_SIGNIFICANT_DIFFERENCE = 0;

    private double preHangP = 0.008;
    private double hangP = 0.035;
    private double currentP = preHangP;




    public Hooks(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "hooks");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetTicks = 0;
    }


    public void setTarget(int ticks) {
        targetTicks = ticks;
    }

    public void setTargetToPreHang() {
        setTarget(1200);
        currentP = preHangP;
    }

    public void setTargetToHang() {
        setTarget(400);
        currentP = hangP;
    }

    public void depower() {
        depowered = true;
    }

    public double currentTicks() {
        return currentTicks;
    }
    public double currentTarget() {
        return targetTicks;
    }

    public void goToTarget() {
        if (targetTicks < 10 && currentTicks < 10) {
            currentPower = 0;
        } else {
            currentPower = (targetTicks - currentTicks) * currentP;
        }
        if (depowered) {
            currentPower = 0;
        }
    }

    public boolean isAtTarget() {
        return RobotMath.isAbsDiffWithinRange(currentTicks, targetTicks, 10);
    }

    public void read() {
        currentTicks = motor.getCurrentPosition();
    }

    public void write() {
        if (Math.abs((previousPower - currentPower)) > POWER_SIGNIFICANT_DIFFERENCE) {
            motor.setPower(currentPower);
        }
        previousPower = currentPower;
    }
}
