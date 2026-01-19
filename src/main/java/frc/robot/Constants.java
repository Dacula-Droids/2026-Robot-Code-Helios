// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  public static class LocalizationConstants{
    public final static double maxSpeedMetersPerSecond = 2.0; //Needs to be tuned

    //Arbitrary
    public final static int leftHubAprilTagID = 100;
    public final static int rightHubAprilTagID = 101;

    //Arbitrary
     public final static Pose3d leftHubPose = new Pose3d(new Translation3d(14.02, 2.03, 0), new Rotation3d(0,0,0)); //Left Tag, Zero for Robot to Face Hub
     public final static Pose3d rightHubPose = new Pose3d(new Translation3d(14.02, 3.61, 0), new Rotation3d(0,0,0)); //Right Tag, Zero for Robot to Face Hub
    
     

  }
}
