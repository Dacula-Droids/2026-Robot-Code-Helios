package frc.robot.Utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public enum IntakePreset {
    Intake(Degrees.of(19)),
    Test(Degrees.of(85)),
    Stowed(Degrees.of(80));
    

    public final Angle position;

    private IntakePreset(Angle position) {
        this.position = position;
    }

}