package org.firstinspires.ftc.teamcode;

public class IntakeArmPosition {
    double armPitchDeg;
    double wristPitchDeg, rollDeg;
    boolean open;

    public IntakeArmPosition(double armPitchDeg, double wristPitchDeg, double rollDeg, boolean open) {
        this.armPitchDeg = armPitchDeg;
        this.wristPitchDeg = wristPitchDeg;
        this.rollDeg = rollDeg;
        this.open = open;
    }
}
