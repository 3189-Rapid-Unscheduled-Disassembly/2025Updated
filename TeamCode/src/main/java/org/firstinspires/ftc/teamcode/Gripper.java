package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.annotation.Nonnull;

public class Gripper {
    Servo gripper;
    private double previousServoPosition;
    final double SERVO_POSITION_SIGNIFICANT_DIFFERENCE = 0.0001;
    double position;

    final double openPos;
    final double closePos;

    String labelTelemetry;


    public Gripper(Servo gripper, double openPos, double closePos, String labelTelemetry) {
       this.gripper = gripper;
        this.openPos = openPos;
        this.closePos = closePos;
        this.labelTelemetry = labelTelemetry;
    }

    public void setPosition(boolean open) {
        if (open) {
            open();
        } else {
            close();
        }
    }
    public void open() {
        setPosition(openPos);
    }
    public void close() {
        setPosition(closePos);
    }

    public void flipFlop() {
        if (isOpen()) {
            close();
        } else {
            open();
        }
    }

    public boolean isOpen() {
        return RobotMath.isAbsDiffWithinRange(position, openPos, 0.001);
    }
    public void readPosition() {
        position = gripper.getPosition();
    }
    public void writePosition() {
        if (Math.abs(previousServoPosition - position) > SERVO_POSITION_SIGNIFICANT_DIFFERENCE) {
            gripper.setPosition(position);
        }
        previousServoPosition = position;
    }
    public void setPosition(double pos) {
        position = pos;
    }


    @Nonnull
    public String toString() {
        return labelTelemetry + " is " + (isOpen() ? "Open" : "Closed");
    }

    @Nonnull
    public String toStringServoPos() {
        return labelTelemetry + String.format(": %.2f Servos", position);
    }

}
