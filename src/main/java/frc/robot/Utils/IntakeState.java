package frc.robot.Utils;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public enum IntakeState {
    STOWED(
            Preset.Stowed.position,
            RPM.of(0)),

    INTAKING(
            Preset.Intake.position,
            RPM.of(600)),

    HOLDING(
            Preset.Intake.position,
            RPM.of(150)),

    OUTTAKING(
            Preset.Intake.position,
            RPM.of(-600));

    public final Angle PivotAngle;
    public final AngularVelocity RollerSpeed;

    private IntakeState(Angle pivotAngle, AngularVelocity rollerSpeed) {
        this.PivotAngle = pivotAngle;
        this.RollerSpeed = rollerSpeed;
    }
}
