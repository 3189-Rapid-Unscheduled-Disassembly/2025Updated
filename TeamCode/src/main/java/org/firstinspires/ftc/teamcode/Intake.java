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

    boolean hasAmpsTriggered = false;


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
        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalSlide = new LinearSlide(horizontalSlideMotor, "horizontalSlide",//61.6667, 51.94979
                45.6, 12, -1, 0.25,
                0.006, 0.18);

        savedPositions.put("transfer", -0.25);//-0.5
        savedPositions.put("max", 12.0);


        intakeArm = new IntakeArm(hardwareMap);

        Servo gateServo = hardwareMap.get(Servo.class, "gate");
        gateServo.setDirection(Servo.Direction.REVERSE);
        gate = new Joint(gateServo, 270, 0.1, "gate");

        timer = new ElapsedTime();
        timer.reset();
    }

    public void changeSavedPositionByInches(String key, double inchChange) {
        double newInches = savedPositions.get(key) + inchChange;
        savedPositions.remove(key);
        savedPositions.put(key, newInches);
    }

    public void closeGate() {
        gate.setAngleDegrees(-27);
    }

    public void partiallyOpenGate() {
        gate.setAngleDegrees(45);
    }

    public void fullyOpenGate() {
        gate.setAngleDegrees(210);
    }
    public void readAllComponents() {
       horizontalSlide.readCurrentPosition();
       horizontalSlide.readCurrentAmps();
    }

    public void firstFrameOfTransfer() {
        double slideDistanceMultipllier = -40;//-36
        if (intakeArm.isPitchEqualToSavedIntakePosition("grabCheck")) {
            waitTimeMS = slideDistanceMultipllier*horizontalSlide.currentInches() + 300;//400
        } else {
            waitTimeMS = slideDistanceMultipllier*horizontalSlide.currentInches() + 400;//450
        }

        setHorizontalSlideToSavedPosition("transfer");

        hasAmpsTriggered = false;

        //reset timer to eliminate weird stuff
        timer.reset();
    }
    //TRANSFERRING
    public boolean transfer(boolean outputIsReady, boolean outputGripperIsOpen) {
        if (timer.milliseconds() > 4000) {
            timer.reset();
        }

        //horizontalSlide.setPower(-0.8);

        //we haven't reached the location, so we need to keep pulling back
        /*if (!hasAmpsTriggered) {
            if (horizontalSlide.isAbovePositionInches(3)) {
                horizontalSlide.setPower(-1);
            } else {
                horizontalSlide.setPower(-0.6);
            }
            if (horizontalSlide.currentAmps() > 4) {
                hasAmpsTriggered = true;
                horizontalSlide.setPower(-0.1);
            }
        } else {
            horizontalSlide.setPower(-0.1);
        }*/

        //this is what happens after the output closes, making us open the intake gripper
        if (!outputGripperIsOpen) {
            //we are ready to open intake and get out of way
            if (timer.milliseconds() > 200) {
                if (timer.milliseconds() > 300) {
                    intakeArm.setToSavedIntakeArmPosition("preTransfer");
                }
                intakeArm.intakeGripper.open();
            }
            return true;
        } else {
            if (!outputIsReady) {
                intakeArm.setToSavedIntakeArmPosition("preTransfer");
            } else {
                intakeArm.setToSavedIntakeArmPosition("transfer");
                if (!isHorizontalSlideAtSavedPos("transfer", 0.5)) {
                    timer.reset();
                } else {
                    if (timer.milliseconds() > waitTimeMS) {
                        timer.reset();
                        horizontalSlide.setPower(0);
                        return true;
                    }
                }
            }
            return false;
            /*if (!isHorizontalSlideAtSavedPos("transfer", 0.5)) {
                intakeArm.setToSavedIntakeArmPosition("transfer");
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
            }*/
        }

        //return false;
    }

    public boolean transferAuto(boolean outputIsReady, boolean outputGripperIsOpen) {
        if (timer.milliseconds() > 4000) {
            timer.reset();
        }

        if (horizontalSlide.isAbovePositionInches(-1)) {
            horizontalSlide.setPower(-0.8);
        } else {
            horizontalSlide.setPower(0);
        }

        //this is what happens after the output closes, making us open the intake gripper
        if (!outputGripperIsOpen) {
            //we are ready to open intake and get out of way
            if (timer.milliseconds() > 200) {
                if (timer.milliseconds() > 300) {
                    intakeArm.setToSavedIntakeArmPosition("preTransfer");
                }
                intakeArm.intakeGripper.open();
            }
            return true;
        } else {
            if (!outputIsReady) {
                intakeArm.setToSavedIntakeArmPosition("preTransfer");
            } else {
                intakeArm.setToSavedIntakeArmPosition("transfer");
                if (!isHorizontalSlideAtSavedPos("transfer", 0.5)) {
                    timer.reset();
                } else {
                    if (timer.milliseconds() > waitTimeMS) {
                        timer.reset();
                        horizontalSlide.setPower(-0.2);
                        return true;
                    }
                }
            }
            return false;
            /*if (!isHorizontalSlideAtSavedPos("transfer", 0.5)) {
                intakeArm.setToSavedIntakeArmPosition("transfer");
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
            }*/
        }

        //return false;
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