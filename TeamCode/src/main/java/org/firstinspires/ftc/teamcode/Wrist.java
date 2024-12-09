package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    Servo left, right;
    private double currentLeftServoPosition;
    private double currentRightServoPosition;
    private double previousLeftServoPosition;
    private double leftServoPosition;
    private double rightServoPosition;
    final double DEGREES_FROM_ZERO_TO_ONE = 355;
    //final double ANGLE_IS_ZERO_AT_THIS_SERVO_POS_LEFT = 0.5;//.5154//.4883
    final double ANGLE_IS_ZERO_AT_THIS_SERVO_POS_LEFT = 0.5154;//.5154//.4883
    final double ANGLE_IS_ZERO_AT_THIS_SERVO_POS_RIGHT= 0.4883;//.5154//.4883
    final double SERVO_POSITION_SIGNIFICANT_DIFFERENCE = 0.0001;



    public Wrist(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "wristLeft");
        right = hardwareMap.get(Servo.class, "wristRight");

        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);
    }


    public double wristPitchCurrent() {
        return  (leftDegrees() + rightDegrees())/2;
    }

    //this will take the left and right servo positions and calculate the wrist's roll
    //in degrees
    public double wristRollCurrent() {
        return (leftDegrees() - rightDegrees());
    }

    /* Everything in Degrees
    1. Determine the average servo position (Pitch)
    2. Determine desired difference in servos (Roll)
    3. Send left and right 1/2 difference above or below
     */
    public void setWristToTarget(double pitchTarget, double rollTarget, double armPitch) {
        pitchTarget -= armPitch;
        pitchTarget = RobotMath.maxAndMin(pitchTarget, 130, -140);
        rollTarget = RobotMath.maxAndMin(rollTarget, 112, 0);
        double leftDeg = pitchTarget + (rollTarget / 2);
        double rightDeg = pitchTarget - (rollTarget / 2);

        leftServoPosition = degToServoLeft(leftDeg);
        rightServoPosition = degToServoRight(rightDeg);
    }


    public double leftDegrees() {
        return RobotMath.servoToDeg(leftServoPosition, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_LEFT, DEGREES_FROM_ZERO_TO_ONE);
    }
    public double rightDegrees() {
        return RobotMath.servoToDeg(rightServoPosition, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_RIGHT, DEGREES_FROM_ZERO_TO_ONE);
    }

    public double degToServoLeft(double degrees) {
        return RobotMath.degToServo(degrees, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_LEFT, DEGREES_FROM_ZERO_TO_ONE, 180, -135);
    }
    public double degToServoRight(double degrees) {
        return RobotMath.degToServo(degrees, ANGLE_IS_ZERO_AT_THIS_SERVO_POS_RIGHT, DEGREES_FROM_ZERO_TO_ONE, 180, -135);
    }

    public void writeServoPositions() {
        if (Math.abs(previousLeftServoPosition - leftServoPosition) > SERVO_POSITION_SIGNIFICANT_DIFFERENCE) {
            left.setPosition(leftServoPosition);
            right.setPosition(rightServoPosition);
        }
        previousLeftServoPosition = leftServoPosition;
    }

    public void readServoPositions() {
        currentLeftServoPosition = left.getPosition();
        currentRightServoPosition = right.getPosition();
    }

    public double getCurrentRightServoPosition() {
        return rightServoPosition;
    }
    public double getCurrentLeftServoPosition() {
        return leftServoPosition;
    }

    public String toString() {
        return String.format("PITCH: %.0f°\nROLL: %.0f°", wristPitchCurrent(), wristRollCurrent());
    }
}
