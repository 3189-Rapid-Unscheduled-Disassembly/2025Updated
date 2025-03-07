package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    LinearSlide verticalSlides;
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


    public Output(HardwareMap hardwareMap, boolean resetEncoders) {
        DcMotorEx front = hardwareMap.get(DcMotorEx.class, "slideFront");
        DcMotorEx back = hardwareMap.get(DcMotorEx.class, "slideBack");

        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.FORWARD);

        if (resetEncoders) {
            back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<DcMotorEx> motors = new ArrayList<>();
        motors.add(back);
        motors.add(front);

        verticalSlides = new LinearSlide(motors, "verticalSlide",
                106.08896551724137931034482758621/*153.829*/, 27, 0, 0.25,
                0.008, 0.18);

        Servo armLeft = hardwareMap.get(Servo.class, "armLeft");
        Servo armRight = hardwareMap.get(Servo.class, "armRight");
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight.setDirection(Servo.Direction.FORWARD);
        List<Servo> armServos = new ArrayList<>();
        armServos.add(armLeft);
        armServos.add(armRight);
        arm = new Joint(armServos, 180, 0.28, "Arm");

        Servo wristServo = hardwareMap.get(Servo.class, "wrist");
        wristServo.setDirection(Servo.Direction.FORWARD);
        wrist = new Joint(wristServo, 300, 0.5, "Wrist Relative to Arm");

        Servo gripperServo = hardwareMap.get(Servo.class, "outputGripper");
        gripperServo.setDirection(Servo.Direction.REVERSE);
        gripper = new Gripper(gripperServo, 0.5, 0.14, "Output Gripper");//0.22


        //put saved positions
        savedPositions.put("aboveTransfer",
                new OutputEndPoint(0, 0, -20, false)
        );
        savedPositions.put("transfer",//2.8, -25.5
                new OutputEndPoint(4, -25.5, -95, true)
        );
        savedPositions.put("rest",
                new OutputEndPoint(0, -15, -90,false)
        );
        savedPositions.put("restOpen",
                new OutputEndPoint(0, -15, -90,true)
        );
        savedPositions.put("straightOut",
                new OutputEndPoint(0, 0, 0,true)
        );
        savedPositions.put("grab",
                new OutputEndPoint(0, -12, -90, true)
        );//-11
        savedPositions.put("aboveGrab",
                new OutputEndPoint(0, -3, -90, false)
        );
        savedPositions.put("highBarFront",
                new OutputEndPoint(7.75, 10, -20, false)
        );
        savedPositions.put("highBarBack",
                new OutputEndPoint(0, 140, 165, false)
        );
        savedPositions.put("highBarBackMoveWrist",
                new OutputEndPoint(0, 140, 140, true)
        );
        savedPositions.put("level1AscentAuto",
                new OutputEndPoint(new Point2d(8.83, 22.5), 0, false)
        );
        savedPositions.put("level1AscentTeleop",
                new OutputEndPoint(new Point2d(10.11, 20),  0, false)
        );
        savedPositions.put("lowBucket",
                new OutputEndPoint(0, 110, 150, false)
        );

        savedPositions.put("highBucket",
                new OutputEndPoint(17.5, 110, 150, false)
        );
        savedPositions.put("highBucketFlat",
                OutputEndPoint.createNewBasedOffOldEndPoint(savedPositions.get("highBucket"),
                        2, true,
                        125, false,
                        0, true,
                        false)
        );
        savedPositions.put("highBucketVertical",
                OutputEndPoint.createNewBasedOffOldEndPoint(savedPositions.get("highBucketFlat"),
                        0, true,
                        0, true,
                        100, false,
                        false)
        );


        savedPositions.put("preHang",
                new OutputEndPoint(14, 20, 20, true)
        );
        savedPositions.put("hang",
                new OutputEndPoint(1, 20, 20, true)
        );

        savedPositions.put("preLevel2",
                new OutputEndPoint(4, 140, 165, true)
        );

        savedPositions.put("preHangArmBackLevel3",
                new OutputEndPoint(24, 140, 165, true)
        );
        savedPositions.put("preHangLevel3",
                new OutputEndPoint(24, 0, 0, true)
        );
        savedPositions.put("hangLevel3",
                new OutputEndPoint(0, 0, 0, true)
        );

        savedPositions.put("fight",
                new OutputEndPoint(0, 40, 40, true)
        );



        timer = new ElapsedTime();
        timer.reset();
    }


    public void readAllComponents() {
        verticalSlides.readCurrentPosition();
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

    public void firstFrameOfTransfer() {
        setComponentPositionsFromSavedPosition("transfer");
    }

    //TRANSFERRING
    public boolean transfer() {
        //setComponentPositionsFromSavedPosition("transfer");

        //if (timer.milliseconds() > 2500) {//3500
            //setComponentPositionsFromSavedPosition("transfer");
//            timer.reset();
        //}

        if (!verticalSlides.isAtTarget()) {
            timer.reset();
        }
        //should check to see if slides are at spot
        return (verticalSlides.isAtTarget());// && timer.milliseconds() > 800);
        /*if (gripper.isOpen()) {
            if (timer.milliseconds() > 1500) {
                gripper.close();
                timer.reset();
            }
        } else {
            if (timer.milliseconds() > 1000) {
                //setComponentPositionsFromSavedPosition("aboveTransfer");
            }

            return timer.milliseconds() > 500;
        }*/

        //return false;
    }


    public void level3Hang() {
        if (verticalSlides.isAbovePositionInches(1)) {
            verticalSlides.setPower(-1);
        } else {
            verticalSlides.setTargetInches(0);
            sendVerticalSlidesToTarget();
        }
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