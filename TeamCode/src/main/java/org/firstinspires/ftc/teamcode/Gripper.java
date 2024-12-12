package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.annotation.Nonnull;

public class Gripper {
    Servo front, back;
    private double positionBack;
    private double positionFront;

    private double currentPosition;
    private double previousServoPosition;
    final double SERVO_POSITION_SIGNIFICANT_DIFFERENCE = 0.0001;
    final double openPosBack = 0.1;
    final double closePosBack = 0.5;
    final double openPosFront = 0.1;
    final double closePosFront = 0.475;


    public Gripper(HardwareMap hardwareMap) {
        front = hardwareMap.get(Servo.class, "gripperFront");
        back = hardwareMap.get(Servo.class, "gripperBack");

        front.setDirection(Servo.Direction.REVERSE);
        back.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(boolean open) {
        if (open) {
            open();
        } else {
            close();
        }
    }
    public void open() {
        setPositionBack(openPosBack);
        setPositionFront(openPosFront);
    }
    public void close() {
        setPositionBack(closePosBack);
        setPositionFront(closePosFront);
    }

    public void flipFlop() {
        if (isOpen()) {
            close();
        } else {
            open();
        }
    }
    public boolean isOpen() {
        return RobotMath.isAbsDiffWithinRange(positionBack, openPosBack, 0.001);
    }
    public void readPosition() {
        currentPosition = front.getPosition();
    }
    public void writePosition() {
        if (Math.abs(previousServoPosition - positionBack) > SERVO_POSITION_SIGNIFICANT_DIFFERENCE) {
            front.setPosition(positionFront);
            back.setPosition(positionBack);
        }
        previousServoPosition = positionBack;
    }
    public void setPositionBack(double pos) {
        positionBack = pos;
    }
    public void setPositionFront(double pos) {
        positionFront = pos;
    }


    @Nonnull
    public String toString() {
        return "Gripper is " + (isOpen() ? "Open" : "Closed");
    }
}
