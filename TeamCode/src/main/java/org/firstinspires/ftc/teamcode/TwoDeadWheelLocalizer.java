package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double parYTicks =  17.443006079436756;//70.08535515751917;//-6402.044144774698; //3.038 in// y position of the parallel encoder (in tick units)
        public double perpXTicks = 7783.355848228351;//7249.670668026067;//5057.833678157886; //2.136 in x position of the perpendicular encoder (in tick units)
        //perpx 1.375
        //pary
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;
    public final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    public double subtractorRad = 0;


    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "fl")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "slideFront")));

        // TODO: reverse encoder directions if needed
//        par.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);


        this.imu = imu;

        this.inPerTick = inPerTick;

      FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    //ADDED
    @Override
    public double getSubtractorRad() {
        return subtractorRad;
    }
    @Override
    public void setSubtractorRad(double rad) {
        subtractorRad = rad;
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        //ADDED, angle subtract for teleop stuff. shoudln't impact auto
        heading.minus(Rotation2d.fromDouble(subtractorRad));


        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
    }
}
