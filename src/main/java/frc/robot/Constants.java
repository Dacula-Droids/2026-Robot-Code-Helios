// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kArmControllerPort = 1;
    public static final double kSwerveControllerDeadband = 0.3;
  }

  public static class PhysicalConstants {
    public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(4.5);
  }

  public static class ArmConstants{

    public static final int kIntakeMotorId  = 20;
    public static final int kEndEffectorMotorId = 21;
    //Change these values later
    public static final double forwardSoftLimit = 10.2;
    public static final double reverseSoftLimit  = 2;

    public static final double intakeMotorSpeed = 0.8;

    public static final double endEffectorTolerance = 2; 

    public static final double outakeMotorSpeed = 0.3;
  }

  public static class ShooterConstants{
    public static final int kShooterMotorId = 8;
    public static final int kOutakeMotorId = 9;


    public static final double shootingOutakeMotorSpeed = 0.7;
    public static final double kShooterForwardSoftLimit = 10;
    public static final double kShooterBackwardSoftLimit = 1;
    public static final double kShooterSuctionLevel = 2;
  }
  
}

