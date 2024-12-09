package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

public class Intake {

    HashMap<String, Double> savedPositions = new HashMap<String, Double>();

    ElapsedTime timer;


    DcMotorEx horizontalSlide;
    IntakeArm intakeArm;

    private double horizontalSlidePower;
    private double previousHorizontalSlidePower;
    final double HORIZ_POWER_SIGNIFICANT_DIFFERENCE = 0.0001;


    //-14 0
    //171 3
    int horizontalSlidePosition;

    final double TICKS_PER_INCH = 61.666667;//175.5
    final double MIN_POINT = 6;
    final double MAX_POINT = 24;
    final double DEFAULT_ALLOWED_ERROR = 0.5;


    public Intake(HardwareMap hardwareMap) {
        horizontalSlide = hardwareMap.get(DcMotorEx.class, "slideHoriz");

        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        savedPositions.put("transfer", MIN_POINT);
        savedPositions.put("max", MAX_POINT);

        horizontalSlidePosition = 0;

        intakeArm = new IntakeArm(hardwareMap);


        timer = new ElapsedTime();
        timer.reset();
    }

    public void readAllComponents() {
        horizontalSlidePosition = horizontalSlide.getCurrentPosition();
    }

    //TRANSFERRING
    public void transferClip() {
        setHorizontalSlideToSavedPosition("transfer");
        intakeArm.setToSavedIntakeArmPosition("transferClip");
    }
    public boolean transferSample() {
        setHorizontalSlideToSavedPosition("transfer");
        intakeArm.setToSavedIntakeArmPosition("transferSample");

        //timer to lower the output once it is there
        if (isAtSavedPosition("transfer")) {
            if (timer.milliseconds() > 3000) {
                timer.reset();
            }
            //this is what we say once we are there
            //should be enough time to put intake arm in right spot
            return timer.milliseconds() > 1000;
        } else {
            timer.reset();
        }
        return false;
    }

    public void setHorizontalSlideToSavedPosition(String key) {
        setHorizontalSlidePositionInches(savedPositions.get(key));
    }

    public boolean isAtSavedPosition(String key) {
        return isAtPosition(savedPositions.get(key), DEFAULT_ALLOWED_ERROR);
    }

    public boolean isAtSavedPosition(String key, double allowedError) {
        return isAtPosition(savedPositions.get(key), allowedError);
    }
    public boolean isAtPosition(double inches) {
        return isAtPosition(inches, DEFAULT_ALLOWED_ERROR);
    }
    public boolean isAtPosition(double inches, double allowedError) {
        return RobotMath.isAbsDiffWithinRange(inches, currentInches(), allowedError);
    }

    public double currentInches() {
        return ticksToInches(horizontalSlidePosition);
    }

    public void setHorizontalSlidePositionInches(double inches) {
        double tickTarget = inchesToTicks(RobotMath.maxAndMin(inches, 24, 6));
        double error = tickTarget - horizontalSlidePosition;

        horizontalSlidePower = 0.006 * error;
    }

    public double inchesToTicks(double inches) {
        return (inches - MIN_POINT) * TICKS_PER_INCH;
    }
    public double ticksToInches(double ticks) {
        return (ticks / TICKS_PER_INCH) + MIN_POINT;
    }


    public void setHorizontalSlidePower(double power) {
        horizontalSlidePower = power;
    }

    public void resetEncoder() {
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //GATE

    //WRITE
    public void writeAllComponents() {
        writeHorizontalSlide();
        intakeArm.writeServoPositions();
    }

    public void writeHorizontalSlide() {
        if (!RobotMath.isAbsDiffWithinRange(previousHorizontalSlidePower, horizontalSlidePower, HORIZ_POWER_SIGNIFICANT_DIFFERENCE)) {
            horizontalSlide.setPower(horizontalSlidePower);
        }
        previousHorizontalSlidePower = horizontalSlidePower;
    }



}
