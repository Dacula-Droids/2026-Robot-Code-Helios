// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 2;
    public static final int kButtonBoardControllerPort = 1;
    public static final double kSwerveControllerDeadband = 0.3;
  }

  public static class PhysicalConstants {
    public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(5);
  }

  public static class IntakeConstants {
    public static final int intakeMotorID = 31;
    public static final int pivotMotorID = 30;
    public static final double intakePivotGearRatio = 18; // 18:1 Gear Ratio
    public static final double intakeRollerGearRatio = 1; // 6:7 Gear Ratio
    public static final Mass intakeMass = Units.Pounds.of(8.7342717); // 8.7342717 lbs
    public static final Distance intakeCenterOfMassFromPivot = Units.Inches.of(9);
    public static final int maxGamePieceCapacity = 50;
    public static final Distance distanceFromRobotCenter = Units.Meters.of(0.3048);
    public static final Distance intakeWidth = Units.Inches.of(26.75);
    public static final Distance intakeExtensionLength = Units.Inches.of(13.8);
    public static final Angle lowerIntakeSoftLimit = Degrees.of(-5);
    public static final Angle upperIntakeSoftLimit = Degrees.of(100);
  }

  public static class IndexerConstants {
    public static final int spinDexerMotorID = 32;
    public static final int kickerMotorID = 33;
    public static final double spinDexerGearRatio = 5;
    public static final double kickerGearRatio = 10;
    public static final AngularVelocity spinDexerVelocity = RPM.of(1000);
    public static final AngularVelocity kickerVelocity = RPM.of(1000);
  }

  public static class ShooterConstants {
    public static final int shooterFlywheelMotorID = 34;
    public static final int shooterPitchMotorID = 35;
    public static final int shooterThroughboreEncoderID = 36;
    public static final double shooterFlywheelGearRatio = 1;
    public static final double shooterPitchGearRatio = 2;
    public static final Angle shooterThroughboreEncoderOffset = Degrees.of(33.25);

  }

  public static class VisionConstants{
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5,0.5,1);
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }
}
