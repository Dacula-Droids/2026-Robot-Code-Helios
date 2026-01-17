// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
  public static class DriveConstants{
    public final static double MAXIMUM_SPEED = Units.feetToMeters(4.5);
  }
  public static class OperatorConstants{
    public final static int DRIVER_CONTROLLER_PORT = 1;
    public final static int MECHANISM_CONTROLLER_PORT = 2;
    public final static double SWERVE_DEADBAND = 0.3;
  }
}
