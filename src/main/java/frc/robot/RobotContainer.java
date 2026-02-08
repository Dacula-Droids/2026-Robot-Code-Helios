// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController mechanismXbox = new CommandXboxController(OperatorConstants.kMechanismControllerPort);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.kSwerveControllerDeadband)
      .scaleTranslation(0.15).scaleRotation(0.15)
      .allianceRelativeControl(false);
  SwerveInputStream driveRobotOrientedAngularVelocity = driveAngularVelocity.copy().robotRelative(true);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveRobotOrientedAngularVelocity);
    driverXbox.b().onTrue(Commands.runOnce(() -> swerveSubsystem.zeroFieldOrientedHeading(driveAngularVelocity), swerveSubsystem));
    //swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    swerveSubsystem.setDefaultCommand(driveRobotOrientedAnglularVelocity);

    if (Robot.isSimulation()) {
      driverXbox.a().onTrue(
          Commands.runOnce(() -> swerveSubsystem.swerveDrive.resetOdometry(new Pose2d(7.6, 1.178, new Rotation2d()))));
    }

    mechanismXbox.a().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.INTAKING));
    mechanismXbox.b().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.HOLDING));
    mechanismXbox.y().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.OUTTAKING));
    mechanismXbox.x().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.STOWED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
}
