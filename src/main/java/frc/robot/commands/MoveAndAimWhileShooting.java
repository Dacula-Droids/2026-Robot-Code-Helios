// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAndAimWhileShooting extends Command {
  private final Supplier<Boolean> shouldShoot;
  private final SwerveInputStream drivingSwerveInputStream;
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

  // final PIDController rotationalPidController = new PIDController(2.8+0.5, 0.00, 0);

  /** Creates a new MoveAndAimWhileShooting. */
  public MoveAndAimWhileShooting(Supplier<Boolean> shouldShoot, SwerveInputStream drivingSwerveInputStream) {
    this.shouldShoot = shouldShoot;
    this.drivingSwerveInputStream = drivingSwerveInputStream;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivingSwerveInputStream.aimWhile(() -> true);
    // rotationalPidController.enableContinuousInput(-180, 180);
    // rotationalPidController.setTolerance(3);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //rotationalPidController.reset();

    Pose2d robotPose = swerveSubsystem.swerveDrive.getPose();
    ChassisSpeeds robotVelocity = swerveSubsystem.swerveDrive.getRobotVelocity(); 
    ChassisSpeeds absoluteFieldRelativeChassisSpeeds = swerveSubsystem.getAbsoluteFieldRelativeChassisSpeeds(robotPose, robotVelocity);
    robotPose = swerveSubsystem.transtalePoseByLatency(robotPose, robotVelocity, 0.3);
    Optional<double[]> shotParams = shooterSubsystem.getShotParams(robotPose, absoluteFieldRelativeChassisSpeeds);

    if (shotParams.isPresent()) {
      Pose2d aimTargetPose2d = 
      new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(shotParams.get()[0]))
      .plus(new Transform2d(1,0, Rotation2d.fromRadians(0)));
      // System.out.println("SHOT PARAMS!!!!!!!!!!!!!!!!!!!::::: " + shotParams.get()[0]);
      //rotationalPidController.setSetpoint(Units.radiansToDegrees(shotParams.get()[0]));

      drivingSwerveInputStream.aim(aimTargetPose2d);
      swerveSubsystem.swerveDrive.driveFieldOriented(drivingSwerveInputStream.get());
      
      
      // swerveSubsystem.swerveDrive.driveFieldOriented(
      //   new ChassisSpeeds(drivingSwerveInputStream.get().vxMetersPerSecond, 
      //   drivingSwerveInputStream.get().vyMetersPerSecond,
      //    Units.degreesToRadians(rotationalPidController.calculate(robotPose.getRotation().getDegrees()))));

      if (shouldShoot.get()){
        double targetSpeedMPS = shotParams.get()[1];
        shooterSubsystem.setFlywheelVelocityMPS(MetersPerSecond.of(targetSpeedMPS));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
