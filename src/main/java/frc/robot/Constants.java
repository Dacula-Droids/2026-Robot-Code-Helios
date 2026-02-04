// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Units;


/** Add your docs here. */
public final class Constants {
  public static class DriveConstants{
  
  }
  public static class OperatorConstants{
    public final static int DRIVER_CONTROLLER_PORT = 1;
    public final static int buttonBoardPort = 2;
    public final static double SWERVE_DEADBAND = 0.3;
  }
  public static class LocalizationConstants{
    public final static double maxSpeedMetersPerSecond = 2.0; //Needs to be tuned

    //Arbitrary
    public final static int leftHubAprilTagID = 100;
    public final static int rightHubAprilTagID = 101;

    //Exact Field Poses
     public final static Pose3d leftHubPose = new Pose3d(new Translation3d(157.79,172.32,44.25), new Rotation3d(0,0,0)); //Left Tag, Zero for Robot to Face Hub
     public final static Pose3d rightHubPose = new Pose3d(new Translation3d(157.79, 158.32, 44.25), new Rotation3d(0,0,0)); //Right Tag, Zero for Robot to Face Hub
  }

  public static class VisionConstants{
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5,0.5,1);
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

   public static class PhysicalConstants {
    public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(4.5);
  }
}

