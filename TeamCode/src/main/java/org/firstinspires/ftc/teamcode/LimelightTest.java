package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.google.gson.annotations.JsonAdapter;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

@TeleOp
public class LimelightTest extends LinearOpMode {

    Limelight3A limelight;

    boolean hasFoundSample = false;

    ElapsedTime timer;


    Double[] previousSampleLockAngles = new Double[5];

    RobotMain bart;

    double SAMPLE_HEIGHT = 1.5;
    double MOUNTING_ANGLE_RADIANS = Math.toRadians(-65);
    double CAMERA_HEIGHT = 7;


    double distFromCameraToSample;

    double slideTarget;



    @Override
    public void runOpMode() throws InterruptedException {

        previousSampleLockAngles[0] = 0.0;
        previousSampleLockAngles[1] = 0.0;
        previousSampleLockAngles[2] = 0.0;
        previousSampleLockAngles[3] = 0.0;
        previousSampleLockAngles[4] = 0.0;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0);
        //5 inches is the goal

        //Bulk Cache Read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        bart = new RobotMain(hardwareMap, telemetry);
        bart.output.setComponentPositionsFromSavedPosition("straightOut");
        bart.intake.intakeArm.setToSavedIntakeArmPosition("limelight");

        timer = new ElapsedTime();
        timer.reset();
        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.a) {
                hasFoundSample = false;
                bart.intake.intakeArm.setToSavedIntakeArmPosition("limelight");
            }

            if (!hasFoundSample) {
                lookingForSample();
            } else {
                goToSample();
            }
            bart.readHubs();

            telemetry.update();
            bart.writeAllComponents();
        }

    }

    private static Point2d cornerToPoint2d(List<Double> corner) {
        return new Point2d(corner.get(0), corner.get(1));
    }
    private static boolean areAllArrayValuesEqualDouble(Double[] array) {
        double checkVal = array[0];
        for (double val : array) {
            if (val != checkVal) {
                return false;
            }
        }
        return true;
    }
    private static double getSampleLockAngle(double limelightAngle) {
        double sampleLockAngle;
        if (limelightAngle < 22.5) {
            sampleLockAngle = 0;
        } else if (limelightAngle < 67.5) {
            sampleLockAngle = -45;
        } else if (limelightAngle < 112.5) {
            sampleLockAngle = 90;
        } else if (limelightAngle < 157.5) {
            sampleLockAngle = 45;
        } else {
            sampleLockAngle = 0;
        }
        return sampleLockAngle;
    }





    public void lookingForSample() {
        bart.intake.horizontalSlide.setTargetInches(0);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.addData("# of colorResults", result.getColorResults().size());


            double angleBetweenDegrees;
            for (LLResultTypes.ColorResult cr : result.getColorResults()) {
                telemetry.addData("# of corners", cr.getTargetCorners().size());

                double limelightAngle;
                if (cr.getTargetCorners().size() == 3) {
                    Point2d pointA = cornerToPoint2d(cr.getTargetCorners().get(0));
                    Point2d pointB = cornerToPoint2d(cr.getTargetCorners().get(1));
                    Point2d pointC = cornerToPoint2d(cr.getTargetCorners().get(2));
                    double aToB = pointA.distanceToOtherPoint(pointB);
                    double aToC = pointA.distanceToOtherPoint(pointC);
                    double bToC = pointB.distanceToOtherPoint(pointC);
                    List<Double> distances = new ArrayList<>();
                    distances.add(aToB);
                    distances.add(aToC);
                    distances.add(bToC);
                    Collections.sort(distances);

                    //a to b is middle
                    if (distances.get(1) == aToB) {
                        limelightAngle = pointA.angleBetweenPoints0To180(pointB);
                    } else if (distances.get(1) == aToC) {
                        limelightAngle = pointA.angleBetweenPoints0To180(pointC);
                    } else {
                        limelightAngle = pointB.angleBetweenPoints0To180(pointC);
                    }

                    //four corners
                } else if (cr.getTargetCorners().size() > 3){
                    Point2d pointA = cornerToPoint2d(cr.getTargetCorners().get(0));
                    Point2d pointB = cornerToPoint2d(cr.getTargetCorners().get(1));
                    Point2d pointC = cornerToPoint2d(cr.getTargetCorners().get(2));
                    Point2d pointD = cornerToPoint2d(cr.getTargetCorners().get(3));
                    double aToB = pointA.distanceToOtherPoint(pointB);
                    double aToC = pointA.distanceToOtherPoint(pointC);
                    double aToD = pointA.distanceToOtherPoint(pointD);

                    List<Double> distances4 = new ArrayList<>();
                    distances4.add(aToB);
                    distances4.add(aToC);
                    distances4.add(aToD);
                    Collections.sort(distances4);

                    //a to b is middle
                    if (distances4.get(1) == aToB) {
                        limelightAngle = pointA.angleBetweenPoints0To180(pointB);
                    } else if (distances4.get(1) == aToC) {
                        limelightAngle = pointA.angleBetweenPoints0To180(pointC);
                    } else {
                        limelightAngle = pointA.angleBetweenPoints0To180(pointD);
                    }


                } else {
                    Point2d pointA = cornerToPoint2d(cr.getTargetCorners().get(0));
                    Point2d pointB = cornerToPoint2d(cr.getTargetCorners().get(1));
                    limelightAngle = pointA.angleBetweenPoints0To180(pointB);

                }


                telemetry.addData("ANGLE", limelightAngle);

                double sampleLockAngle = getSampleLockAngle(limelightAngle);

                telemetry.addData("sampleLockAngle", sampleLockAngle);

                //kick back the array values
                previousSampleLockAngles[4] = previousSampleLockAngles[3];
                previousSampleLockAngles[3] = previousSampleLockAngles[2];
                previousSampleLockAngles[2] = previousSampleLockAngles[1];
                previousSampleLockAngles[1] = previousSampleLockAngles[0];
                previousSampleLockAngles[0] = sampleLockAngle;


            }

            for (double angle2 : previousSampleLockAngles) {
                telemetry.addData("val", angle2);
            }
            //set the roll if the last 5 frames are all the same
            if (areAllArrayValuesEqualDouble(previousSampleLockAngles)) {
                bart.intake.intakeArm.setOnlySpecifiedValuesToIntakeArmPosition(new IntakeArmPosition(0,0, previousSampleLockAngles[0], true),
                        false, false, true, false);
                telemetry.addLine("MOVE WRIST");
                if (gamepad1.b) {
                    hasFoundSample = true;
                    slideTarget = bart.intake.horizontalSlide.currentInches()+distFromCameraToSample-3.5;
                }
            }





            //Estimate Distance
            distFromCameraToSample = (SAMPLE_HEIGHT - CAMERA_HEIGHT) / (Math.tan(MOUNTING_ANGLE_RADIANS + Math.toRadians(tx)));
            distFromCameraToSample = RobotMath.maxAndMin(distFromCameraToSample, 10, 0);

            telemetry.addData("Distance", distFromCameraToSample);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

    }


    public void goToSample() {
        bart.intake.horizontalSlide.setTargetInches(slideTarget);
        bart.intake.horizontalSlide.goToTargetAsync();

        if (bart.intake.horizontalSlide.isAtTarget()) {
            if (timer.milliseconds() > 2000) {
                bart.intake.intakeArm.setOnlySpecifiedValuesToSavedIntakeArmPosition("grab",
                        true, true, false, true);
            } else {
                bart.intake.intakeArm.setOnlySpecifiedValuesToSavedIntakeArmPosition("preGrab",
                        true, true, false, true);
            }
        } else {
            timer.reset();
        }
    }
}
