package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

//this holds the pitches, roll, and gripper for the intake
public class IntakeArm {
    Servo intakeGripper, intakePitchLeft, intakePitchRight, intakeRoll;

    HashMap<String, IntakeArmPosition> savedPositions = new HashMap<String, IntakeArmPosition>();



    final double DEGREES_FROM_ZERO_TO_ONE_PITCH = 270;
    final double ANGLE_IS_ZERO_AT_THIS_SERVO_POS_PITCH = 0.5;

    //left is positive
    final double DEGREES_FROM_ZERO_TO_ONE_ROLL = 270;
    final double ANGLE_IS_ZERO_AT_THIS_SERVO_POS_ROLL = 0.52;

    private double pitchPosDeg;
    private double previousPitchPosDeg = 200;
    final double PITCH_POS_SIGNIFICANT_DIFFERENCE = 0.0001;


    private double rollPosDeg;
    private double previousRollPosDeg = 200;
    final double ROLL_POS_SIGNIFICANT_DIFFERENCE = 0.0001;

    private double gripperPosServo;
    private double previousGripperPosServo = 2;
    final double GRIPPER_POS_SIGNIFICANT_DIFFERENCE = 0.0001;


    double gripperOpenPos = 0.15;
    double gripperClosePos = 0.5;

    double theNew90Pitch = 100;


    public IntakeArm(HardwareMap hardwareMap) {
        intakePitchLeft = hardwareMap.get(Servo.class, "intakePitchLeft");
        intakePitchRight = hardwareMap.get(Servo.class, "intakePitchRight");
        intakeRoll = hardwareMap.get(Servo.class, "intakeRoll");
        intakeGripper = hardwareMap.get(Servo.class, "intakeGripper");

        intakePitchLeft.setDirection(Servo.Direction.REVERSE);
        intakePitchRight.setDirection(Servo.Direction.FORWARD);
        intakeRoll.setDirection(Servo.Direction.FORWARD);
        intakeGripper.setDirection(Servo.Direction.FORWARD);

        savedPositions.put("transferClip", new IntakeArmPosition(35,0, false));
        savedPositions.put("transferSample", new IntakeArmPosition(100, 90, false));
        savedPositions.put("preGrab", new IntakeArmPosition(-72, 0, true));
        savedPositions.put("grab", new IntakeArmPosition(-82, 0, true));
    }


    //these are for teleop. dpad can turn the roll
    public void moveRollPositive45() {
        if (rollPosDeg < -45) {
            setRollDeg(-45);
        } else if (rollPosDeg < 0) {
            setRollDeg(0);
        } else if (rollPosDeg < 45) {
            setRollDeg(45);
        } else if (rollPosDeg < 90) {
            setRollDeg(90);
        } else {
            setRollDeg(-45);
        }
    }
    public void moveRollNegative45() {
        if (rollPosDeg > 45) {
            setRollDeg(45);
        } else if (rollPosDeg > 0) {
            setRollDeg(0);
        } else if (rollPosDeg > -45) {
            setRollDeg(-45);
        } else if (rollPosDeg > -90) {
            setRollDeg(-90);
        } else {
            setRollDeg(45);
        }
    }

    //this cycles between transfer, preGrab, grab, and then back to transfer
    //this allows us to quickly cycle to the desired pitch
    public void cycle() {
        if (isPitchEqualToSavedIntakePosition("transferSample")) {
            setToSavedIntakeArmPosition("preGrab");
        } else if (isPitchEqualToSavedIntakePosition("preGrab")) {
            setToSavedIntakeArmPosition("grab");
        } else {
            setToSavedIntakeArmPosition("transferSample");
        }
    }

    //this is just for checking if we are at a certain pitch
    //we ignore roll/gripper so that way we can roll and then we still know what saved pos we're at
    public boolean isPitchEqualToSavedIntakePosition(String key) {
        return isPitchEqualToIntakeArmPosition(savedPositions.get(key));
    }
    public boolean isPitchEqualToIntakeArmPosition(IntakeArmPosition intakeArmPosition) {
        return RobotMath.isAbsDiffWithinRange(pitchPosDeg, intakeArmPosition.pitchDeg, 0.0001);
    }

    public void setToSavedIntakeArmPosition(String key) {
        setToIntakeArmPosition(savedPositions.get(key));
    }
    public void setToIntakeArmPosition(IntakeArmPosition intakeArmPosition) {
        setPitchDeg(intakeArmPosition.pitchDeg);
        setRollDeg(intakeArmPosition.rollDeg);
        setGripperPosition(intakeArmPosition.open);
    }


    public double servoToDegPitch(double inputServoPosition) {
        return RobotMath.servoToDeg(inputServoPosition, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_PITCH, DEGREES_FROM_ZERO_TO_ONE_PITCH);
    }
    public double degToServoPitch(double degrees) {
        return RobotMath.degToServo(degrees, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_ROLL, DEGREES_FROM_ZERO_TO_ONE_ROLL, 135, -135);
    }

    public double servoToDegRoll(double inputServoPosition) {
        return RobotMath.servoToDeg(inputServoPosition, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_ROLL, DEGREES_FROM_ZERO_TO_ONE_ROLL);
    }
    public double degToServoRoll(double degrees) {
        return RobotMath.degToServo(degrees, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_ROLL, DEGREES_FROM_ZERO_TO_ONE_ROLL, 90, -90);
    }

    //set each one directly
    public void setPitchDeg(double pitchDeg) {
        pitchPosDeg = pitchDeg;
    }
    public void setRollDeg(double rollDeg) {
        rollPosDeg = rollDeg;
    }

    //GRIPPER STUFF
    public void flipFlop() {
        if (isOpen()) {
            close();
        } else {
            open();
        }
    }
    public void setGripperPosition(boolean open) {
        if (open) {
            open();
        } else {
            close();
        }
    }
    public void open() {
        gripperPosServo = gripperOpenPos;
    }
    public void close() {
        gripperPosServo = gripperClosePos;
    }
    public boolean isOpen() {
        return RobotMath.isAbsDiffWithinRange(gripperPosServo, gripperOpenPos, 0.001);
    }



    public double getPitchPosDeg() {
        return pitchPosDeg;
    }
    public double getPitchPosServo() {
        return degToServoPitch(pitchPosDeg);
    }
    public double getRollPosDeg() {
        return rollPosDeg;
    }
    public double getRollPosServo() {
        return degToServoRoll(rollPosDeg);
    }


    public void writeServoPositions() {
        writePitch();
        writeRoll();
        writeGripper();
    }

    public void writePitch() {
        if (!RobotMath.isAbsDiffWithinRange(previousPitchPosDeg, pitchPosDeg, PITCH_POS_SIGNIFICANT_DIFFERENCE)) {
            intakePitchLeft.setPosition(degToServoPitch(pitchPosDeg));
            intakePitchRight.setPosition(degToServoPitch(pitchPosDeg));
        }
        previousPitchPosDeg = pitchPosDeg;
    }

    public void writeRoll() {
        if (!RobotMath.isAbsDiffWithinRange(previousRollPosDeg, rollPosDeg, ROLL_POS_SIGNIFICANT_DIFFERENCE)) {
            intakeRoll.setPosition(degToServoRoll(rollPosDeg));
        }
        previousRollPosDeg = rollPosDeg;
    }

    public void writeGripper() {
        if (!RobotMath.isAbsDiffWithinRange(previousGripperPosServo, gripperPosServo, GRIPPER_POS_SIGNIFICANT_DIFFERENCE)) {
            intakeGripper.setPosition(gripperPosServo);
        }
        previousGripperPosServo = gripperPosServo;
    }

    //985g
    @Override
    public String toString() {
        return String.format("\nPitch: %.0f°", getPitchPosDeg()) +
                String.format("\nRoll: %.0f°", getRollPosDeg()) +
                "\nGripper is " + (isOpen() ? "open" : "closed");
    }

    public String posServoTelemetry() {
        return String.format("\nPitch: %.2f Servos", getPitchPosServo()) +
                String.format("\nRoll: %.2f Servos", getRollPosServo());
    }
}
