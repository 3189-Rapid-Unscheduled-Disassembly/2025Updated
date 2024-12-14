package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

/*this class should replace the arm and wrist classes
and make the intakeArm class simpler

Joint is designed to be a joint that can only rotate in one plane.
The Arm for example, or the non-differential wrist

As of 2024-12-14, on the way to Casper, I think we will have 4 Joint objects:
Output: Arm, Wrist
Intake: Pitch, Roll

Hopefully this class wil just streamline the process, because we currently have a lot of very similar things that could be put into one class
Don't think we implement this class until we use the non-differential wrist

This class is designed to use either a servo or multiple servos that are all set to the same position always (like the arm)
 */
public class Joint {
    List<Servo> servos;

    String labelTelemetry;

    double currentPositionAngleDegrees;
    double previousPositionAngleDegrees;

    final double DEGREES_SIGNIFICANT_DIFFERENCE = 0.0001;
    final double DEGREES_FROM_ZERO_TO_ONE;//[-45,135]
    final double ANGLE_IS_ZERO_AT_THIS_SERVO_POS;


    public Joint(List<Servo> servos,
                 double DEGREES_FROM_ZERO_TO_ONE,
                 double ANGLE_IS_ZERO_AT_THIS_SERVO_POS,
                 String labelTelemetry) {
        this.servos = servos;
        this.DEGREES_FROM_ZERO_TO_ONE = DEGREES_FROM_ZERO_TO_ONE;
        this. ANGLE_IS_ZERO_AT_THIS_SERVO_POS = ANGLE_IS_ZERO_AT_THIS_SERVO_POS;
        this.labelTelemetry = labelTelemetry;
    }

    public Joint(Servo servo,
                 double DEGREES_FROM_ZERO_TO_ONE,
                 double ANGLE_IS_ZERO_AT_THIS_SERVO_POS,
                 String labelTelemetry) {
        this.servos = new ArrayList<>();
        this.servos.add(servo);

        this.DEGREES_FROM_ZERO_TO_ONE = DEGREES_FROM_ZERO_TO_ONE;
        this.ANGLE_IS_ZERO_AT_THIS_SERVO_POS = ANGLE_IS_ZERO_AT_THIS_SERVO_POS;
        this.labelTelemetry = labelTelemetry;
    }

    public void setAngleDegrees(double degrees) {
        currentPositionAngleDegrees = degrees;
    }

    //this is used by the wrist to set the angle relative to the ground, no matter what the arm angle is
    public void setAngleDegreesMinusOtherAngle(double degrees, double otherDegrees) {
        setAngleDegrees(degrees - otherDegrees);
    }

    public double currentAngleDegrees() {
        return currentPositionAngleDegrees;
    }

    public double servoToDeg(double servoPos) {
        return RobotMath.servoToDeg(servoPos, ANGLE_IS_ZERO_AT_THIS_SERVO_POS, DEGREES_FROM_ZERO_TO_ONE);
    }

    public double degToServo(double degrees) {
        return RobotMath.degToServo(degrees, ANGLE_IS_ZERO_AT_THIS_SERVO_POS, DEGREES_FROM_ZERO_TO_ONE, 180, -180);
    }


    //this actually sets the hardware positions
    private void setPosition() {
        double servoPos = degToServo(currentPositionAngleDegrees);
        for (Servo servo : servos) {
            servo.setPosition(servoPos);
        }
    }

    public void write() {
        if (Math.abs(previousPositionAngleDegrees - currentPositionAngleDegrees) > DEGREES_SIGNIFICANT_DIFFERENCE) {
            setPosition();
        }
        previousPositionAngleDegrees = currentPositionAngleDegrees;
    }

    public String toString() {
        return labelTelemetry + String.format(": %.0f°", currentPositionAngleDegrees);
    }
}
