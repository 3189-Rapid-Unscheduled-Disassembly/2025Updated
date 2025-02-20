package org.firstinspires.ftc.teamcode;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
ports
2025-01-07
CONTROL
MOTORS
0 fl
1 fr
2 slideFront
3 steve (this is for the roadrunner encoder)
SERVOS
0 intakeGripper
1 intakeRoll
2 intakeWristPitch
3 (armLeft was in here, but it didn't work, maybe it's the port's problem)
4 intakeArmPitch
5

EXPANSION
MOTORS
0 bl
1 br
2 slideHoriz
3 slideBack
SERVOS
0 outputGripper
1 wrist
2 armRight
3 armLeft
4
5 gate
*/

public class RobotMain {

    Telemetry telemetry = null;

    MecanumDrive mecanaDruve = null;
    MecanumDrive roadrunnerMecanum;

    Output output = null;
    Intake intake = null;

    boolean robotCentric = true;


    //FIELD POINTS AND LINES
    final static FieldPoint2d BUCKET_POINT = new FieldPoint2d(new Point2d(66, 66));
    final static FieldPoint2d CLIP_PLACE_POINT = new FieldPoint2d(new Point2d(-37, 70));
    final static FieldXLine CLIP_BAR = new FieldXLine(12, -12, 23);
    final static FieldYLine HANG_BAR = new FieldYLine(14.5, 20, -20);


    public RobotMain(HardwareMap hardwareMap, Telemetry telemetry) {
        //Receive Hardware Map and telemetry
        this.telemetry = telemetry;

        //imu = new RevIMU(hardwareMap);
        //imu.init();



        // input motors exactly as shown below
        mecanaDruve = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //DcMotorEx slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        //DcMotorEx slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        //Servo armLeft = hardwareMap.get(Servo.class, "armLeft");
        //Servo armRight = hardwareMap.get(Servo.class, "armRight");

        //Servo wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        //Servo wristRight = hardwareMap.get(Servo.class, "wristRight");
        //Servo gripperFront = hardwareMap.get(Servo.class, "gripperFront");
        //Servo gripperBack = hardwareMap.get(Servo.class, "gripperBack");

        //wristLeft.setDirection(Servo.Direction.FORWARD);
        //wristRight.setDirection(Servo.Direction.REVERSE);//club test2
        //club test fr5

        //armLeft.setDirection(Servo.Direction.REVERSE);
        //armRight.setDirection(Servo.Direction.FORWARD);

        //gripperFront.setDirection(Servo.Direction.REVERSE);
        //gripperBack.setDirection(Servo.Direction.FORWARD);

        //VerticalSlides verticalSlides = new VerticalSlides(hardwareMap);
        //Arm arm = new Arm(hardwareMap);
        //Wrist wrist = new Wrist(hardwareMap);
        //Gripper gripper = new Gripper(hardwareMap);


        output = new Output(hardwareMap);
        intake = new Intake(hardwareMap);

    }

    public void switchDriveMode() {
        robotCentric = !robotCentric;
    }

    public void resetEncoders() {
        output.verticalSlides.resetEncoder();
        intake.horizontalSlide.resetEncoder();
    }

    //public void resetIMU() {
    //imu.reset();
    //}


    /*public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        if (robotCentric) {
            mecanaDruve.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
        } else {
            mecanaDruve.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, imu.getHeading());
        }

        telemetry.addData("Forward Speed:", forwardSpeed);
        telemetry.addData("Strafe Speed: ", strafeSpeed);
        telemetry.addData("Turn Speed: ", turnSpeed);
    }*/

    //this really just determines if we are doing a close or far transfer
    public void firstFrameOfTransfer() {
       intake.firstFrameOfTransfer();
       output.firstFrameOfTransfer();
    }

    public void transfer() {
        boolean outputIsReady = output.transfer();
        boolean outputShouldGrab = intake.transfer(outputIsReady, output.gripper.isOpen());
        if (outputShouldGrab) {
            output.gripper.close();
        }
    }



    public void driveRobotRelative(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        mecanaDruve.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        forwardSpeed,
                        strafeSpeed
                ),
                turnSpeed
        ));
    }
    public void driveFieldRelative(double xSpeed, double ySpeed, double turnSpeed) {
        double cos = Math.cos(mecanaDruve.pose.heading.toDouble());
        double sin = Math.sin(mecanaDruve.pose.heading.toDouble());

        double robotForwardSpeed = xSpeed*cos + ySpeed*sin;
        double robotStrafeSpeed = -xSpeed*sin + ySpeed*cos;

        driveRobotRelative(robotForwardSpeed, robotStrafeSpeed, turnSpeed);
    }

    public Point2d robotPosAsPoint2d() {
        return new Point2d(mecanaDruve.pose.position.x, mecanaDruve.pose.position.y);
    }


    public void printTelemetry() {
        //telemetry.addData("Global Angle: ", imu.getAbsoluteHeading());
        //telemetry.addData("Offset Angle: ", imu.getHeading());
        telemetry.addData("Driver Mode Is Robot Centric? ", robotCentric);

        telemetry.update();
    }

    public void readHubs() {
        output.readAllComponents();
        intake.readAllComponents();
    }

    public void resetGyro(double angleRobotIsAtDeg) {
        mecanaDruve.setPosFromOutside(new Pose2d(new Vector2d(mecanaDruve.pose.position.x, mecanaDruve.pose.position.y), Math.toRadians(angleRobotIsAtDeg)));
    }

    public static double dpadInputToChangeValueUpIsPositive(double currentValue, GamepadEx gamepadEx, double increment) {
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            return currentValue - 6*increment;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            return currentValue + 6*increment;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            return currentValue + increment;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            return currentValue - increment;
        }
        return currentValue;
    }

    public static double dpadInputToChangeValueUpIsNegative(double currentValue, GamepadEx gamepadEx) {
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            return currentValue + 6;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            return currentValue - 6;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            return currentValue - 1;
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            return currentValue + 1;
        }
        return currentValue;
    }

    public void writeAllComponents() {
        output.writeAllComponents();
        intake.writeAllComponents();
    }
}