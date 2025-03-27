package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

public class LinearSlide {
    List<DcMotorEx> motors;

    String labelTelemetry;

    double currentError;
    double previousError;

    double previousTime;
    double deltaTime;
    double derivative;


    boolean specialMode = false;

    boolean hasAmpsTriggeredForEncoderReset = false;
    double timeWhenAmpsTriggeredMS = 0;


    private double currentTicks = -3600;
    private double targetTicks = -3600;

    private double currentAmps = 0;

    private double currentPower = -3600;
    private double previousPower = -3600;

    final double SLIDE_POWER_SIGNIFICANT_DIFFERENCE = 0;

    final double DEFAULT_ERROR_INCHES;

    final double TICKS_PER_INCH;
    final double p, d;

    final double MAX_INCHES, MIN_INCHES;
    final double MAX_TICKS, MIN_TICKS;

    ElapsedTime pidTimer;

    public LinearSlide(List<DcMotorEx> motors,
                       String labelTelemetry,
                       double TICKS_PER_INCH,
                       double MAX_INCHES,
                       double MIN_INCHES,
                       double DEFAULT_ERROR_INCHES,
                       double p, double d) {
        this.motors = motors;

        this.TICKS_PER_INCH = TICKS_PER_INCH;
        this.labelTelemetry = labelTelemetry;

        this.MAX_INCHES = MAX_INCHES;
        this.MIN_INCHES = MIN_INCHES;
        this.MAX_TICKS = inchesToTicks(MAX_INCHES);
        this.MIN_TICKS = inchesToTicks(MIN_INCHES);

        this.DEFAULT_ERROR_INCHES = DEFAULT_ERROR_INCHES;

        this.p = p;
        this.d = d;

        pidTimer = new ElapsedTime();
        pidTimer.reset();

        previousError = 0;
        previousTime = 0;
    }

    public LinearSlide(DcMotorEx motor,
                       String labelTelemetry,
                       double TICKS_PER_INCH,
                       double MAX_INCHES,
                       double MIN_INCHES,
                       double DEFAULT_ERROR_INCHES,
                       double p, double d) {
        this.motors = new ArrayList<>();
        this.motors.add(motor);

        this.TICKS_PER_INCH = TICKS_PER_INCH;
        this.labelTelemetry = labelTelemetry;

        this.MAX_INCHES = MAX_INCHES;
        this.MIN_INCHES = MIN_INCHES;
        this.MAX_TICKS = inchesToTicks(MAX_INCHES);
        this.MIN_TICKS = inchesToTicks(MIN_INCHES);

        this.DEFAULT_ERROR_INCHES = DEFAULT_ERROR_INCHES;

        this.p = p;
        this.d = d;

        pidTimer = new ElapsedTime();
        pidTimer.reset();
    }

    public void setTargetTicks(double ticks) {
        ticks = RobotMath.maxAndMin(ticks, MAX_TICKS, MIN_TICKS);
        targetTicks = ticks;
    }
    public void setTargetInches(double inches) {
        setTargetTicks(inchesToTicks(inches));
    }
    public void setTargetToCurrentPosition() {
        targetTicks = currentTicks();
    }


    public boolean isAtTarget() {
        return isAtTarget(DEFAULT_ERROR_INCHES);
    }

    public boolean isAtTarget(double allowedError) {
        return RobotMath.isAbsDiffWithinRange(currentInches(), ticksToInches(targetTicks), allowedError);
    }
    public boolean isAtPositionInches(double inches) {
        return isAtPositionInches(inches, DEFAULT_ERROR_INCHES);
    }
    public boolean isAtPositionInches(double inches, double allowedError) {
        return RobotMath.isAbsDiffWithinRange(currentInches(), inches, allowedError);
    }

    public boolean isAbovePositionInches(double inches) {
        return isAbovePositionTicks(inchesToTicks(inches));
    }

    public boolean isAbovePositionTicks(double ticks) {
        return currentTicks() > ticks;
    }
    public boolean isAboveMax() {
        return isAbovePositionInches(MAX_INCHES);
    }

    public boolean isBellowMin() {
        return !isAbovePositionInches(MIN_INCHES);
    }

    public void setSpecialMode(boolean specialMode) {
        this.specialMode = specialMode;
    }

    public void goToTargetAsync() {
        currentError = targetTicks - currentTicks();

        deltaTime = pidTimer.milliseconds() - previousTime;
        previousTime = pidTimer.milliseconds();
        derivative = (currentError - previousError) / deltaTime;
        previousError = currentError;
        //we don't need to power when the slides are at zero

        if (motors.size() == 1 || (targetTicks > 10 || currentTicks() > 10)) {
            //if (!isAtTarget()) {
            setPower((currentError * p) + (derivative * d));
            //} else {
            //setSlidePower(currentTicks() * 0.000003);//maybe need back drive in the future
            //slidePower = 0;
            //}
        } else {
          setPower(0);
        }

    }




    public void resetEncoder() {
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double ticksToInches(double ticks) {
        return ticks/TICKS_PER_INCH;
    }
    public double inchesToTicks(double inches) {
        return inches*TICKS_PER_INCH;
    }

    public double currentInches() {
        return ticksToInches(currentTicks());
    }
    public double currentTicks() {
        return currentTicks;
    }

    public double currentAmps() {
        return currentAmps;
    }


    //this is used to reset both slides at the start of teleop
    //it'll send both down until the voltage spikes
    //then return true once both are done
    public boolean voltageResetEncoder() {
        if (!hasAmpsTriggeredForEncoderReset) {
            setPower(-0.6);
            if (currentAmps > 4) {
                hasAmpsTriggeredForEncoderReset = true;
                setPower(0);
                timeWhenAmpsTriggeredMS = pidTimer.milliseconds();
            }
            return false;
        } else {
            setPower(0);
            double timeDifferenceMS = pidTimer.milliseconds() - timeWhenAmpsTriggeredMS;
            if (timeDifferenceMS > 500) {
                resetEncoder();
                return true;
            }
            return false;
        }
    }

    public void setPower(double power) {
        currentPower = power;
    }

    public void readCurrentPosition() {
        currentTicks = motors.get(0).getCurrentPosition();
        //dumb, just needed for horiz transfer
        //specialMode = false;
    }
    public void readCurrentAmps() {
        currentAmps = motors.get(0).getCurrent(CurrentUnit.AMPS);
    }
    public void writeSlidePower() {
        if (Math.abs((previousPower - currentPower)) > SLIDE_POWER_SIGNIFICANT_DIFFERENCE) {
            for (DcMotorEx motor : motors) {
                motor.setPower(currentPower);
            }
        }
        previousPower = currentPower;
    }





}
