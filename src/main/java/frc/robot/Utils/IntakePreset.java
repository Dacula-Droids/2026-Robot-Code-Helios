package frc.robot.Utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public enum IntakePreset {
    Intake(Degrees.of(-20)),
    Test(Degrees.of(-50)),
    Stowed(Degrees.of(0));
    

    public final Angle position;

    private IntakePreset(Angle position) {
        this.position = position;
    }

}