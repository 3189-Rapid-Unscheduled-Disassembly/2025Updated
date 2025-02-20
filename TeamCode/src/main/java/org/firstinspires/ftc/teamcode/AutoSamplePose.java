package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class AutoSamplePose {
    private double color, x, y, roll;
    private final double xIncrement = 1;
    private final double yIncrement = 1;
    private final double rollIncrement = 45;

    private final boolean isYellowLegal, isBlueLegal, isRedLegal;

    private final double xMax, xMin, yMax, yMin, rollMax, rollMin;

    boolean stillInputting = true;

    //color: 0 = yellow, 1 = blue, 2 = red
    //x, y are as if you are blue. Left is always +x, towards you is always +y
    //roll is relative to how the robot will be grabbing it. So, on the bucket, it'll be rotated from the drivers' perspective
    public AutoSamplePose(double color, double x, double y, double roll,
                          boolean isYellowLegal, boolean isBlueLegal, boolean isRedLegal,
                          double xMax, double xMin, double yMax, double yMin, double rollMax, double rollMin) {
        this.color = color;
        this.x = x;
        this.y = y;
        this.roll = roll;

        this.isYellowLegal = isYellowLegal;
        this.isBlueLegal = isBlueLegal;
        this.isRedLegal = isRedLegal;

        this.xMax = xMax;
        this.xMin = xMin;

        this.yMax = yMax;
        this.yMin = yMin;

        this.rollMax = rollMax;
        this.rollMin = rollMin;
    }

    public void inputAutoSamplePose(GamepadEx gamepadEx) {
        //COLOR INPUT
        if (isYellowLegal && gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
            setColor(0);
        }
        if (isBlueLegal && gamepadEx.wasJustPressed(GamepadKeys.Button.X)) {
            setColor(1);
        }
        if (isRedLegal && gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            setColor(2);
        }

        //X INPUT
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            changeXLeft();
        }
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            changeXRight();
        }
        x = RobotMath.maxAndMin(x, xMax, xMin);

        //Y INPUT
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            changeYDown();
        }
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            changeYUp();
        }
        y = RobotMath.maxAndMin(y, yMax, yMin);


        //ROLL INPUT
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            changeRollLeft();
        }
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            changeRollRight();
        }
        roll = RobotMath.maxAndMin(roll, rollMax, rollMin);



        //SIGNAL WE'RE DONE
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            doneInputting();
        }

    }


    //COLOR
    public double getColor() {
        return color;
    }
    public void setColor(double color) {
        this.color = color;
    }

    //X
    public double getX() {
        return x;
    }
    public void changeXLeft() {
        this.x += xIncrement;
    }
    public void changeXRight() {
        this.x -= xIncrement;
    }

    //Y
    public double getY() {
        return y;
    }
    public void changeYDown() {
        this.y += yIncrement;
    }
    public void changeYUp() {
        this.y -= yIncrement;
    }

    //ROLL
    public double getRoll() {
        return roll;
    }
    public void changeRollLeft() {
        this.roll += rollIncrement;
    }
    public void changeRollRight() {
        this.roll -= rollIncrement;
    }

    //DONE INPUTTING
    public void doneInputting() {
        stillInputting = false;
    }


    @NonNull
    @Override
    public String toString() {
        String colorString = "BLANK";
        if (color == 0) {
            colorString = "YELLOW";
        } else if (color == 1) {
            colorString = "BLUE";
        } else if (color == 2) {
            colorString = "RED";
        }

        String rollString = "BLANK";
        if (roll == 0) {
            rollString = "STRAIGHT";
        } else if (roll == -45) {
            rollString = "RIGHT";
        } else if (roll == 45) {
            rollString = "LEFT";
        } else if (roll == 90) {
            rollString = "THOGGIN'";
        }

        return colorString + "\nX: " + x + "\nY: " + y + "\nROLL: " + roll +"\n" + rollString;
    }

}
