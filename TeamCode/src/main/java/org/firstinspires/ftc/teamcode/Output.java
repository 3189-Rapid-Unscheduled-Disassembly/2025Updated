package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/*PLAN
1. Points
    - Arm Point
        - trig
    - Slide Point
        - equations of line and circle
        - quadratic
        - use x1 or x2
2. Calculate the hardware positions
3. Go to those positions
 */

public class Output {
    VerticalSlides verticalSlides;
    Joint arm;
    Joint wrist;
    Gripper gripper;

    ElapsedTime timer;

    HashMap <String, OutputEndPoint> savedPositions = new HashMap<String, OutputEndPoint>();


    double slideTargetInches;
    double armTargetPitchDegrees;
    double wristTargetPitchRelativeToGroundDegrees;
    boolean open;


    //VALUES
    //possible slide values: 9.44882 (240mm) times 2
    //slide angle (relative to straight forward): 100
    //slide start point (-5.56, 15.171)
    //arm length: 13.3
    //wrist length: 3.268


    public Output(HardwareMap hardwareMap) {
        verticalSlides = new VerticalSlides(hardwareMap);

        Servo armLeft = hardwareMap.get(Servo.class, "armLeft");
        Servo armRight = hardwareMap.get(Servo.class, "armRight");
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight.setDirection(Servo.Direction.FORWARD);
        List<Servo> armServos = new ArrayList<>();
        armServos.add(armLeft);
        armServos.add(armRight);
        arm = new Joint(armServos, 180, 0.286, "Arm");

        Servo wristServo = hardwareMap.get(Servo.class, "wrist");
        wristServo.setDirection(Servo.Direction.REVERSE);
        wrist = new Joint(wristServo, 300, 0.55, "Wrist Relative to Arm");

        Servo gripperServo = hardwareMap.get(Servo.class, "outputGripper");
        gripperServo.setDirection(Servo.Direction.REVERSE);
        gripper = new Gripper(gripperServo, 0.5, 0.1, "Output Gripper");


        //put saved positions
        savedPositions.put("aboveTransfer",
                new OutputEndPoint(0, 0, -20, false)
        );
        savedPositions.put("transfer",
                new OutputEndPoint(0, -20, -50, true)
        );
        savedPositions.put("rest",
                new OutputEndPoint(0, -15, -90,false)
        );
        savedPositions.put("straightOut",
                new OutputEndPoint(0, 0, 0,true)
        );
        savedPositions.put("grab",
                new OutputEndPoint(0, -13, -90, true)
        );
        savedPositions.put("aboveGrab",
                new OutputEndPoint(0, -3, -90, false)
        );
        savedPositions.put("highBarFront",
                new OutputEndPoint(7, 10, -20, false)
        );
        savedPositions.put("highBarBack",
                new OutputEndPoint(0, 130, 170, false)
        );
        savedPositions.put("level1AscentAuto",
                new OutputEndPoint(new Point2d(8.83, 22.5), 0, false)
        );
        savedPositions.put("level1AscentTeleop",
                new OutputEndPoint(new Point2d(10.11, 20),  0, false)
        );
        savedPositions.put("lowBucket",
                new OutputEndPoint(new Point2d(-12.4, 28),  150, false)
        );
        savedPositions.put("highBucket",
                new OutputEndPoint(new Point2d(-14, 46), 150, false)
        );


        timer = new ElapsedTime();
        timer.reset();
    }


    public void readAllComponents() {
        verticalSlides.readCurrentTicks();
        //arm.readServoPositions();
        //wrist.readServoPositions();
        //gripper.readPosition();
    }

    public void writeAllComponents() {
        verticalSlides.writeSlidePower();
        arm.write();
        wrist.write();
        gripper.writePosition();
    }

    //TRANSFERRING
    public boolean transfer() {
        if (timer.milliseconds() > 3000) {
            setComponentPositionsFromSavedPosition("transfer");
            timer.reset();
        }

        //should check to see if slides are at spot
        if (gripper.isOpen()) {
            if (timer.milliseconds() > 1500) {
                gripper.close();
                timer.reset();
            }
        } else {
            if (timer.milliseconds() > 1000) {
                setComponentPositionsFromSavedPosition("aboveTransfer");
            }

            return timer.milliseconds() > 500;
        }

        return false;
    }



    public void setComponentPositionsFromSavedPosition(String key) {
        setComponentPositionsFromOutputEndPoint(savedPositions.get(key));
    }
    public void setComponentPositionsFromOutputEndPoint(OutputEndPoint outputEndPoint) {
        slideTargetInches = outputEndPoint.slideInches;
        armTargetPitchDegrees = outputEndPoint.armDegrees;
        wristTargetPitchRelativeToGroundDegrees = outputEndPoint.wristPitchRelativeToGroundDegrees;

        open = outputEndPoint.open;
        //set servo positions
        arm.setAngleDegrees(armTargetPitchDegrees);
        wrist.setAngleDegreesMinusOtherAngle(wristTargetPitchRelativeToGroundDegrees, armTargetPitchDegrees);
        gripper.setPosition(open);
        //vertical slides
        verticalSlides.setTargetInches(slideTargetInches);
    }
    public void sendVerticalSlidesToTarget() {
        //slide per frame
        verticalSlides.goToTargetAsync();
        //arm
        //arm.goToAngle(armTargetPitchDegrees);
        //wrist
        //wrist.setWristToTarget(wristTargetPitchRelativeToGroundDegrees, wristTargetRollDegrees, armTargetPitchDegrees);
    }

    public boolean isAtPosition() {
        return verticalSlides.isAtTarget();
    }

    public OutputEndPoint currentPosition() {
        return new OutputEndPoint(
                verticalSlides.currentInches(),
                arm.currentAngleDegrees(),
                arm.currentAngleDegrees() + wrist.currentAngleDegrees(),//gives us the pitch relative to ground
                gripper.isOpen());
    }

    public void setTargetToCurrentPosition() {
        setComponentPositionsFromOutputEndPoint(currentPosition());
    }


}
