package frc.robot.Utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public enum Preset {
    Intake(Degrees.of(-20)),
    Stowed(Degrees.of(10)),
    Shooting(Degrees.of(28.6));

    public final Angle position;

    private Preset(Angle position) {
        this.position = position;
    }

}