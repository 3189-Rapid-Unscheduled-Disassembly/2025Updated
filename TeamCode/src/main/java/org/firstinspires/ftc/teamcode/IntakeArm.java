package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

//this holds the pitches, roll, and gripper for the intake
public class IntakeArm {


    boolean currentlyAttemptingGrab = false;
    ElapsedTime grabTimer;

    Joint intakeArm;
    Joint intakeWristPitch;
    Joint intakeWristRoll;

    Gripper intakeGripper;
    HashMap<String, IntakeArmPosition> savedPositions = new HashMap<String, IntakeArmPosition>();

    public IntakeArm(HardwareMap hardwareMap) {

        Servo armLeftServo = hardwareMap.get(Servo.class,"intakeArmPitchLeft");
        Servo armRightServo = hardwareMap.get(Servo.class,"intakeArmPitchRight");
        armLeftServo.setDirection(Servo.Direction.FORWARD);
        armRightServo.setDirection(Servo.Direction.REVERSE);

        List<Servo> armServos = new ArrayList<>();
        armServos.add(armLeftServo);
        armServos.add(armRightServo);
        intakeArm = new Joint(armServos, 200, 0.200, "Intake Arm Pitch");

        Servo intakeWristPitchServo = hardwareMap.get(Servo.class, "intakeWristPitch");
        intakeWristPitchServo.setDirection(Servo.Direction.FORWARD);
        intakeWristPitch = new Joint(intakeWristPitchServo, 637.5, 0.458, "Intake Wrist Pitch");

        Servo intakeRollServo = hardwareMap.get(Servo.class, "intakeRoll");
        intakeRollServo.setDirection(Servo.Direction.FORWARD);
        intakeWristRoll = new Joint(intakeRollServo, 270, 0.18, "Intake Roll");

        Servo gripperServo = hardwareMap.get(Servo.class, "intakeGripper");
        gripperServo.setDirection(Servo.Direction.REVERSE);
        intakeGripper = new Gripper(gripperServo, 0.90, 0.495, "Intake Gripper");//0.85, 0.47

        //SAVED POSITIONS
        savedPositions.put("rest", new IntakeArmPosition(125, 180, 0, true));

        savedPositions.put("straightOut", new IntakeArmPosition(0, 0, 0, true));
        double transferRollDeg = 185;//185
        savedPositions.put("preTransfer", new IntakeArmPosition(30, 110, transferRollDeg, false));
        savedPositions.put("transfer", new IntakeArmPosition(48, 180, transferRollDeg, false));//43
        savedPositions.put("drop", new IntakeArmPosition(0, -60,45, false));
        savedPositions.put("preGrab", new IntakeArmPosition(5, -80, 0, true));
        savedPositions.put("grab", new IntakeArmPosition(-15, -90, 0, false));
        savedPositions.put("grabCheck", new IntakeArmPosition(30, -60, 0, false));
        savedPositions.put("postGrab", new IntakeArmPosition(30, 0, 0, false));
        savedPositions.put("stupidGrab", new IntakeArmPosition(38, 0, 90, true));
        savedPositions.put("lowBar", new IntakeArmPosition(18, 80, 90, false));
        savedPositions.put("fight", new IntakeArmPosition(80, 80, 0, true));
        savedPositions.put("limelight", new IntakeArmPosition(15, -15, 0, true));

        savedPositions.put("park", new IntakeArmPosition(80, 125, 0, true));
        savedPositions.put("parkClips", new IntakeArmPosition(60, 100, 0, true));

        grabTimer = new ElapsedTime();
        grabTimer.reset();
    }


    //these are for teleop. dpad can turn the roll
    public void moveRollPositive45() {
        double currentRoll = intakeWristRoll.currentAngleDegrees();
        double targetRoll;
        if (currentRoll < -45) {
            targetRoll = -45;
        } else if (currentRoll < 0) {
            targetRoll = 0;
        } else if (currentRoll < 45) {
            targetRoll = 45;
        } else if (currentRoll < 90) {
            targetRoll = 90;
        } else if (currentRoll == 90){
            targetRoll = -45;
        } else {
            targetRoll = 45;
        }
        intakeWristRoll.setAngleDegrees(targetRoll);
    }
    public void moveRollNegative45() {
        double currentRoll = intakeWristRoll.currentAngleDegrees();
        double targetRoll;
        //this is for weird transfer pre rolling
        if (currentRoll > 90) {
            targetRoll = -45;
        } else if (currentRoll == 90) {
            targetRoll = 45;
        } else if (currentRoll > 0) {
            targetRoll = 0;
        } else if (currentRoll > -45) {
            targetRoll = -45;
        } else {
            targetRoll = 90;
        }
        intakeWristRoll.setAngleDegrees(targetRoll);
    }

    //this cycles between transfer, preGrab, grab, and then back to transfer
    //this allows us to quickly cycle to the desired pitch
    public void cycle() {
        if (isPitchEqualToSavedIntakePosition("preGrab")) {
            currentlyAttemptingGrab = true;
            grabTimer.reset();
            setOnlySpecifiedValuesToSavedIntakeArmPosition("grab", true, true, false, false);

        } else if (isPitchEqualToSavedIntakePosition("grab")){
            setOnlySpecifiedValuesToSavedIntakeArmPosition("grabCheck", true, true, false, true);
        } else if (isPitchEqualToSavedIntakePosition("grabCheck") ||
                    isPitchEqualToSavedIntakePosition("postGrab")){
            setOnlySpecifiedValuesToSavedIntakeArmPosition("preGrab", true, true, false, true);
        } else {
            //used only for the start of teleop when we started with a sample already in grabber
            if (intakeArm.isAngleEqualToGivenAngle(-3600)) {
                setOnlySpecifiedValuesToSavedIntakeArmPosition("grabCheck", true, true, false, true);
            } else {
                //this is for going from preTransfer or whatever
                setToSavedIntakeArmPosition("preGrab");
            }
        }
    }

    //use this if currently attempting a grab
    //we wanna delay the closing
    public void delayClosingGrabber() {
        if (currentlyAttemptingGrab) {
            if (grabTimer.milliseconds() > 100) {
                intakeGripper.close();
                currentlyAttemptingGrab = false;
            }
        }
    }

    //this is just for checking if we are at a certain pitch
    //we ignore roll/gripper so that way we can roll and then we still know what saved pos we're at
    public boolean isPitchEqualToSavedIntakePosition(String key) {
        return intakeArm.isAngleEqualToGivenAngle(savedPositions.get(key).armPitchDeg);
    }

    public void setToSavedIntakeArmPosition(String key) {
        setToIntakeArmPosition(savedPositions.get(key));
    }
    public void setToIntakeArmPosition(IntakeArmPosition intakeArmPosition) {
        intakeArm.setAngleDegrees(intakeArmPosition.armPitchDeg);
        intakeWristPitch.setAngleDegrees(intakeArmPosition.wristPitchDeg);
        intakeWristRoll.setAngleDegrees(intakeArmPosition.rollDeg);
        intakeGripper.setPosition(intakeArmPosition.open);
    }

    //kinda wierd but we only set the values that are true in the parameters.
    //that we we don't have to set the roll or whatever
    public void setOnlySpecifiedValuesToIntakeArmPosition(IntakeArmPosition intakeArmPosition,
                                                          boolean setArmPitch, boolean setWristPitch, boolean setRoll, boolean setGripper) {
        if (setArmPitch) {
            intakeArm.setAngleDegrees(intakeArmPosition.armPitchDeg);
        }
        if (setWristPitch) {
            intakeWristPitch.setAngleDegrees(intakeArmPosition.wristPitchDeg);
        }
        if (setRoll) {
            intakeWristRoll.setAngleDegrees(intakeArmPosition.rollDeg);
        }
        if (setGripper) {
            intakeGripper.setPosition(intakeArmPosition.open);
        }
    }
    public void setOnlySpecifiedValuesToSavedIntakeArmPosition(String key,
                                                               boolean setArmPitch, boolean setWristPitch, boolean setRoll, boolean setGripper) {
        setOnlySpecifiedValuesToIntakeArmPosition(savedPositions.get(key), setArmPitch, setWristPitch, setRoll, setGripper);
    }



    public void writeServoPositions() {
        intakeWristPitch.write();
        intakeArm.write();
        intakeWristRoll.write();
        intakeGripper.writePosition();
    }

    //985g
    @Override
    public String toString() {
        return intakeArm.toString() +
                "\n" + intakeWristPitch.toString() +
                "\n" + intakeWristRoll.toString() +
                "\n" + intakeGripper.toString();
    }

    public String posServoTelemetry() {
        return intakeArm.toStringServoPos() +
                "\n" + intakeWristPitch.toStringServoPos() +
                "\n" + intakeWristRoll.toStringServoPos() +
                "\n" + intakeGripper.toStringServoPos();
    }
}
