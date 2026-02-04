package frc.robot.Utils;

public enum ApproachMode {
    CloseOutpost(0.45),
    FarOutpost(1);

    public final double offsetValue;

    ApproachMode(double offsetValue) {
            this.offsetValue = offsetValue;
    }
}
