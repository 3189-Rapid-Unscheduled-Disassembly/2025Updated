package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;

public class Intake {

    HashMap<String, Double> savedPositions = new HashMap<String, Double>();

    ElapsedTime timer;


    LinearSlide horizontalSlide;
    IntakeArm intakeArm;
    Joint gate;


    ElapsedTime dTimer;

    double waitTimeMS = 1000;

    final double MIN_POINT = 0;
    final double MAX_POINT = 14;


    public Intake(HardwareMap hardwareMap, boolean resetEncoders) {
        DcMotorEx horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, "slideHoriz");

        horizontalSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (resetEncoders) {
            horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalSlide = new LinearSlide(horizontalSlideMotor, "horizontalSlide",
                61.666667, 14, 0, 0.25,
                0.007, 0.17);

        savedPositions.put("transfer", 0.0);
        savedPositions.put("max", 14.0);


        intakeArm = new IntakeArm(hardwareMap);

        Servo gateServo = hardwareMap.get(Servo.class, "gate");
        gateServo.setDirection(Servo.Direction.REVERSE);
        gate = new Joint(gateServo, 270, 0.1, "gate");

        timer = new ElapsedTime();
        timer.reset();
    }

    public void closeGate() {
        gate.setAngleDegrees(0);
    }

    public void partiallyOpenGate() {
        gate.setAngleDegrees(60);
    }

    public void fullyOpenGate() {
        gate.setAngleDegrees(210);
    }
    public void readAllComponents() {
       horizontalSlide.readCurrentPosition();
    }

    public void firstFrameOfTransfer() {
        if (intakeArm.isPitchEqualToSavedIntakePosition("grabCheck")) {
            waitTimeMS = -36*horizontalSlide.currentInches() + 800;
        } else {
            waitTimeMS = -33.333 * horizontalSlide.currentInches() + 800;
        }

        setHorizontalSlideToSavedPosition("transfer");

        //reset timer to eliminate weird stuff
        timer.reset();
    }
    //TRANSFERRING
    public boolean transfer(boolean outputIsReady, boolean outputGripperIsOpen) {
        if (timer.milliseconds() > 4000) {
            timer.reset();
        }

        //this is what happens after the output closes, making us open the intake gripper
        if (!outputGripperIsOpen) {
            //we are ready to open intake and get out of way
            if (timer.milliseconds() > 200) {
                if (timer.milliseconds() > 400) {
                    intakeArm.setToSavedIntakeArmPosition("preTransfer");
                }
                intakeArm.intakeGripper.open();
            }
            return true;
        } else {
            //the horizontal slide is NOT far enough away to begin moving the arm
            if (!isHorizontalSlideAtSavedPos("transfer", 2)) {
                intakeArm.setToSavedIntakeArmPosition("preTransfer");
                timer.reset();
            } else {
                if (!outputIsReady) {
                    intakeArm.setToSavedIntakeArmPosition("preTransfer");
                    timer.reset();
                } else {
                    intakeArm.setToSavedIntakeArmPosition("transfer");

                    if (timer.milliseconds() > waitTimeMS) {//500
                        timer.reset();
                        return true;
                    }

                }
            }
        }

        return false;

        //tell the output to grab
    }

    public void setHorizontalSlideToSavedPosition(String key) {
        horizontalSlide.setTargetInches(savedPositions.get(key));
    }


    public boolean isHorizontalSlideAtSavedPos(String key) {
        return isHorizontalSlideAtSavedPos(key, horizontalSlide.DEFAULT_ERROR_INCHES);
    }
    public boolean isHorizontalSlideAtSavedPos(String key, double allowedError) {
        return horizontalSlide.isAtPositionInches(savedPositions.get(key), allowedError);
    }

    //WRITE
    public void writeAllComponents() {
        horizontalSlide.writeSlidePower();
        intakeArm.writeServoPositions();
        gate.write();
        //reset horiz encoder if it hits the back stop
    }



}