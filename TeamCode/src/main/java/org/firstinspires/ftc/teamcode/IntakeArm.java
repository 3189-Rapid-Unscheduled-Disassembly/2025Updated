package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

//this holds the pitches, roll, and gripper for the intake
public class IntakeArm {

    Joint intakePitch;
    Joint intakeRoll;

    Gripper intakeGripper;
    HashMap<String, IntakeArmPosition> savedPositions = new HashMap<String, IntakeArmPosition>();

    public IntakeArm(HardwareMap hardwareMap) {
        Servo intakePitchLeft = hardwareMap.get(Servo.class, "intakePitchLeft");
        Servo intakePitchRight = hardwareMap.get(Servo.class, "intakePitchRight");
        intakePitchLeft.setDirection(Servo.Direction.REVERSE);
        intakePitchRight.setDirection(Servo.Direction.REVERSE);
        List<Servo> pitchServos = new ArrayList<>();
        pitchServos.add(intakePitchLeft);
        pitchServos.add(intakePitchRight);
        intakePitch = new Joint(pitchServos, 150, 0.778, "Intake Pitch");

        Servo intakeRollServo = hardwareMap.get(Servo.class, "intakeRoll");
        intakeRollServo.setDirection(Servo.Direction.FORWARD);
        intakeRoll = new Joint(intakeRollServo, 270, 0.52, "Intake Roll");

        Servo gripperServo = hardwareMap.get(Servo.class, "intakeGripper");
        gripperServo.setDirection(Servo.Direction.REVERSE);
        intakeGripper = new Gripper(gripperServo, 0.75, 0.17, "Intake Gripper");

        //SAVED POSITIONS
        savedPositions.put("transfer", new IntakeArmPosition(0,0, false));
        savedPositions.put("drop", new IntakeArmPosition(-60,45, false));
        savedPositions.put("grab", new IntakeArmPosition(-90, 0, true));//0.233,-75
    }


    //these are for teleop. dpad can turn the roll
    public void moveRollPositive45() {
        double currentRoll = intakeRoll.currentAngleDegrees();
        double targetRoll;
        if (currentRoll < -45) {
            targetRoll = -45;
        } else if (currentRoll < 0) {
            targetRoll = 0;
        } else if (currentRoll < 45) {
            targetRoll = 45;
        } else if (currentRoll < 90) {
            targetRoll = 90;
        } else {
            targetRoll = -45;
        }
        intakeRoll.setAngleDegrees(targetRoll);
    }
    public void moveRollNegative45() {
        double currentRoll = intakeRoll.currentAngleDegrees();
        double targetRoll;
        if (currentRoll > 45) {
            targetRoll = 45;
        } else if (currentRoll > 0) {
            targetRoll = 0;
        } else if (currentRoll > -45) {
            targetRoll = -45;
        } else if (currentRoll > -90) {
            targetRoll = -90;
        } else {
            targetRoll = 45;
        }
        intakeRoll.setAngleDegrees(targetRoll);
    }

    //this cycles between transfer, preGrab, grab, and then back to transfer
    //this allows us to quickly cycle to the desired pitch
    public void cycle() {
        if (isPitchEqualToSavedIntakePosition("transfer")) {
            setOnlySpecifiedValuesToSavedIntakeArmPosition("grab", true, false, true);
        } else {
            setToSavedIntakeArmPosition("transfer");
        }
    }

    //this is just for checking if we are at a certain pitch
    //we ignore roll/gripper so that way we can roll and then we still know what saved pos we're at
    public boolean isPitchEqualToSavedIntakePosition(String key) {
        return intakePitch.isAngleEqualToGivenAngle(savedPositions.get(key).pitchDeg);
    }

    public void setToSavedIntakeArmPosition(String key) {
        setToIntakeArmPosition(savedPositions.get(key));
    }
    public void setToIntakeArmPosition(IntakeArmPosition intakeArmPosition) {
        intakePitch.setAngleDegrees(intakeArmPosition.pitchDeg);
        intakeRoll.setAngleDegrees(intakeArmPosition.rollDeg);
        intakeGripper.setPosition(intakeArmPosition.open);
    }

    //kinda wierd but we only set the values that are true in the parameters.
    //that we we don't have to set the roll or whatever
    public void setOnlySpecifiedValuesToIntakeArmPosition(IntakeArmPosition intakeArmPosition,
                                                          boolean setPitch, boolean setRoll, boolean setGripper) {
        if (setPitch) {
            intakePitch.setAngleDegrees(intakeArmPosition.pitchDeg);
        }
        if (setRoll) {
            intakeRoll.setAngleDegrees(intakeArmPosition.rollDeg);
        }
        if (setGripper) {
            intakeGripper.setPosition(intakeArmPosition.open);
        }
    }
    public void setOnlySpecifiedValuesToSavedIntakeArmPosition(String key,
                                                               boolean setPitch, boolean setRoll, boolean setGripper) {
        setOnlySpecifiedValuesToIntakeArmPosition(savedPositions.get(key), setPitch, setRoll, setGripper);
    }



    public void writeServoPositions() {
        intakePitch.write();
        intakeRoll.write();
        intakeGripper.writePosition();
    }

    //985g
    @Override
    public String toString() {
        return intakePitch.toString() +
                "\n" + intakeRoll.toString() +
                "\n" + intakeGripper.toString();
    }

    public String posServoTelemetry() {
        return intakePitch.toStringServoPos() +
                "\n" + intakeRoll.toStringServoPos() +
                "\n" + intakeGripper.toStringServoPos();
    }
}
