package frc.robot.Utils;

import edu.wpi.first.math.util.Units;

public enum Preset{
    Intake(12.4),
    Stowed(24.8),
    Shooting(28.6);
    

    public final double position;

private Preset(double position){
this.position = position;
}

}