package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VerticalSlides {

    DcMotorEx front, back;
    int currentTicks;
    double currentError;
    double previousError;
    double targetTicks;
    double slidePower;
    double previousSlidePower;
    double currentTime;
    double previousTime;
    double deltaTime;
    double derivative;
    final double SLIDE_POWER_SIGNIFICANT_DIFFERENCE = 0;
    ElapsedTime time;

    final double ALLOWED_ERROR_INCHES = 0.25;
    final double p = 0.0033;
    final double d = 0.0015;

    final double TICKS_PER_INCH = 153.829;
    public VerticalSlides(HardwareMap hardwareMap) {
        front = hardwareMap.get(DcMotorEx.class, "slideFront");
        back = hardwareMap.get(DcMotorEx.class, "slideBack");

        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.FORWARD);

        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        previousSlidePower = 0;
        previousError = 0;
        currentTime = 0;
        previousTime = 0;

        time = new ElapsedTime();

    }


    public void resetEncoder() {
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double ticksToInches(double ticks) {
        return ticks/TICKS_PER_INCH;
    }
    public double inchesToTicks(double inches) {
        return inches*TICKS_PER_INCH;
    }

    public void setTargetTicks(int ticks) {
        targetTicks = ticks;
    }
    public void setTargetInches(double inches) {
        targetTicks = inchesToTicks(inches);
    }
    public void setTargetToCurrentPosition() {
        targetTicks = currentTicks();
    }

    //currently just a p controller
    public void goToTargetAsync() {
        currentError = targetTicks - currentTicks();
        currentTime = time.milliseconds();
        deltaTime = currentTime - previousTime;
        derivative = (currentError - previousError) / deltaTime;
        previousError = currentError;
        previousTime = currentTime;
        //we don't need to power when the slides are at zero
        if (targetTicks > 10 || currentTicks > 10) {
            //if (!isAtTarget()) {
                slidePower = (currentError * p) + 0;//(derivative * d);
            //} else {
                //setSlidePower(currentTicks() * 0.000003);//maybe need back drive in the future
                //slidePower = 0;
            //}
        } else {
            slidePower = 0;
        }

    }

    public boolean isAtTarget() {
        return RobotMath.isAbsDiffWithinRange(currentInches(), ticksToInches(targetTicks), ALLOWED_ERROR_INCHES);
    }

    public boolean isAtPositionInches(double inches) {
        return RobotMath.isAbsDiffWithinRange(currentInches(), inches, ALLOWED_ERROR_INCHES);
    }


    public Point2d currentSlidePoint() {
        double x = Math.acos(Math.toRadians(100))/ currentInches();
        double y = Math.asin(Math.toRadians(100))/ currentInches();
        return new Point2d(x, y);
    }

    public boolean isAbovePositionInches(double inches) {
        return isAbovePositionTicks(inchesToTicks(inches));
    }
    public boolean isAbovePositionTicks(double ticks) {
        return currentTicks() > ticks;
    }

    public double currentInches() {
        return ticksToInches(currentTicks());
    }
    public int currentTicks() {
        return currentTicks;
    }

    public void readCurrentTicks() {
        currentTicks = back.getCurrentPosition();
    }
    public void setSlidePower(double power) {
        slidePower = power;
    }
    public void writeSlidePower() {
        if (Math.abs((previousSlidePower - slidePower)) > SLIDE_POWER_SIGNIFICANT_DIFFERENCE) {
            front.setPower(slidePower);
            back.setPower(slidePower);
        }
        previousSlidePower = slidePower;
    }
}
