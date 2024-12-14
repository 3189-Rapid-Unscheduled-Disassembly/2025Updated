package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

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
    Arm arm;
    Wrist wrist;
    Gripper gripper;

    ElapsedTime timer;

    HashMap <String, OutputEndPoint> savedPositions = new HashMap<String, OutputEndPoint>();


    double slideTargetInches;
    double armTargetPitchDegrees;
    double wristTargetPitchRelativeToGroundDegrees;
    double wristTargetRollDegrees;
    boolean open;


    //VALUES
    //possible slide values: 9.44882 (240mm) times 2
    //slide angle (relative to straight forward): 100
    //slide start point (-5.56, 15.171)
    //arm length: 13.3
    //wrist length: 3.268


    public Output(HardwareMap hardwareMap) {
        verticalSlides = new VerticalSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);

        Servo gripperServo = hardwareMap.get(Servo.class, "outputGripper");
        gripperServo.setDirection(Servo.Direction.REVERSE);
        gripper = new Gripper(gripperServo, 0.5, 0.1);

        double theNew90 = 112;

        //put saved positions


        savedPositions.put("aboveTransfer",
                new OutputEndPoint(0, 0, -20, theNew90, false)
        );
        savedPositions.put("transfer",
                new OutputEndPoint(0, -20, -50, theNew90, true)
        );

        savedPositions.put("rest",
                new OutputEndPoint(0, -15, 90, 0, false)
        );
        savedPositions.put("grab",
                new OutputEndPoint(0, -5, -60, theNew90, true)
        );
        savedPositions.put("aboveGrab",
                new OutputEndPoint(0, 10, -60, theNew90, false)
        );
        savedPositions.put("grab2",
                new OutputEndPoint(0, -3, 60, theNew90, true)
        );
        savedPositions.put("highBarFront",
                //new OutputEndPoint(new Point2d(10,25), 40, theNew90, false)
                new OutputEndPoint(0, 35, 40, theNew90, false)
        );
        savedPositions.put("highBarBack",
                new OutputEndPoint(0, 130, 215, theNew90, false)
        );
        savedPositions.put("highBar",
                new OutputEndPoint(new Point2d(8.8, 25.5), 45, theNew90, false)
        );
        savedPositions.put("level1AscentAuto",
                new OutputEndPoint(new Point2d(8.83, 22.5), 0, theNew90, false)
        );
        savedPositions.put("level1AscentTeleop",
                new OutputEndPoint(new Point2d(10.11, 20), 0, 0, false)
        );
        savedPositions.put("lowBucket",
                new OutputEndPoint(new Point2d(-12.4, 28), 180, 0, false)
        );
        savedPositions.put("highBucket",
                new OutputEndPoint(new Point2d(-14, 46), 180, 0, false)
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
        arm.writeServoPositions();
        wrist.writeServoPositions();
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
        wristTargetRollDegrees = outputEndPoint.roll;
        open = outputEndPoint.open;
        //set servo positions
        arm.goToAngle(armTargetPitchDegrees);
        wrist.setWristToTarget(wristTargetPitchRelativeToGroundDegrees, wristTargetRollDegrees, armTargetPitchDegrees);
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
                arm.armAngleCurrent(),
                arm.armAngleCurrent() + wrist.wristPitchCurrent(),//gives us the pitch relative to ground
                wrist.wristRollCurrent(),
                gripper.isOpen());
    }

    public void setTargetToCurrentPosition() {
        setComponentPositionsFromOutputEndPoint(currentPosition());
    }


}
