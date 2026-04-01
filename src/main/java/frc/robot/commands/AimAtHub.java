// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtHub extends Command {

  private final SwerveInputStream drivingSwerveInputStream;
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

  private Rotation2d targetHeading;
  /** Creates a new AimAtHub. */
  public AimAtHub(SwerveInputStream drivingSwerveInputStream) {
    this.drivingSwerveInputStream = drivingSwerveInputStream;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivingSwerveInputStream.aimWhile(() -> true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = swerveSubsystem.getAllianceBasedPose(swerveSubsystem.swerveDrive.getPose());
    ChassisSpeeds robotVelocity = swerveSubsystem.swerveDrive.getRobotVelocity(); 
    ChassisSpeeds absoluteFieldRelativeChassisSpeeds = swerveSubsystem.getAbsoluteFieldRelativeChassisSpeeds(robotPose, robotVelocity);
    //robotPose = swerveSubsystem.transtalePoseByLatency(robotPose, robotVelocity, 0.1);
    Optional<double[]> shotParams = shooterSubsystem.getShotParams(robotPose, robotVelocity);

    if (shotParams.isPresent()) {
      Pose2d aimTargetPose2d = 
      new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(shotParams.get()[0]))
      .plus(new Transform2d(1,0, Rotation2d.fromRadians(0)));
      // System.out.println("SHOT PARAMS!!!!!!!!!!!!!!!!!!!::::: " + shotParams.get()[0]);
      //rotationalPidController.setSetpoint(Units.radiansToDegrees(shotParams.get()[0]));

      drivingSwerveInputStream.aim(aimTargetPose2d);
      targetHeading = Rotation2d.fromRadians(shotParams.get()[0]);
      swerveSubsystem.swerveDrive.driveFieldOriented(drivingSwerveInputStream.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    Pose2d robotPose = swerveSubsystem.swerveDrive.getPose();
    Rotation2d heading = robotPose.getRotation();
    return Math.abs(heading.minus(targetHeading).getDegrees()) < 1;
  }
}
