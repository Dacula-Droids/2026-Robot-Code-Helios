// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Utils.IntakeState;
import frc.robot.commands.IntakingPivot;
import frc.robot.commands.TestPivot;
import frc.robot.commands.ZeroPivot;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  private final IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final KickerSubsystem kickerSubsystem = KickerSubsystem.getInstance();

  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController mechanismXbox = new CommandXboxController(
      OperatorConstants.kMechanismControllerPort);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.kSwerveControllerDeadband)
      .scaleTranslation(0.15).scaleRotation(0.15)
      .allianceRelativeControl(false);
  SwerveInputStream driveRobotOrientedAngularVelocity = driveAngularVelocity.copy().robotRelative(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureAutonomous() {
    DriverStation.silenceJoystickConnectionWarning(true);

    Command goToLeftOutpostAndShoot = new SequentialCommandGroup(
        swerveSubsystem.pathfindToFieldTarget(OutpostTarget.Left, true), Commands.waitSeconds(3),
        getShootingCommand());

    Command goToRightOutpostAndShoot = new SequentialCommandGroup(
        swerveSubsystem.pathfindToFieldTarget(OutpostTarget.Right, true), Commands.waitSeconds(3),
        getShootingCommand());

    autoChooser.setDefaultOption("GoToLeftOutpostAndShoot", goToLeftOutpostAndShoot);
    autoChooser.addOption("GoToRightOutpostAndShoot", goToRightOutpostAndShoot);

    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveRobotOrientedAngularVelocity);

    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // Commands.runOnce(() ->
    // swerveSubsystem.zeroFieldOrientedHeading(driveAngularVelocity),
    // swerveSubsystem));
    // // swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // swerveSubsystem.setDefaultCommand(driveRobotOrientedAnglularVelocity);

    // if (Robot.isSimulation()) {
    // driverXbox.a().onTrue(
    // Commands.runOnce(() -> swerveSubsystem.swerveDrive.resetOdometry(new
    // Pose2d(7.6, 1.178, new Rotation2d()))));
    // }
    // driverXbox.y().whileTrue(intakeSubsystem.setState(IntakeState.STOWED));
    // driverXbox.x().whileTrue(intakeSubsystem.setState(IntakeState.INTAKING));
    // driverXbox.leftBumper().whileTrue(
    // new ParallelCommandGroup(
    // indexerSubsystem.setSpinDexerDutyCycle(0.1)));

    // kickerSubsystem.setKickerDutyCycle(1),
    // shooterSubsystem.setShooterFlywheelDutyCycle(1)));
    // driverXbox.rightBumper().whileTrue(intakeSubsystem.setRollerDutyCycle(0.2));
    // 7-(30*0.149)-0.5)
    // driverXbox.rightBumper().whileTrue(Commands.runOnce(() ->
    // shooterSubsystem.setShooterVoltage( ()));
    driverXbox.rightBumper().onTrue(getShootingCommand());
    driverXbox.leftBumper().onTrue(stopShootingCommand());
    driverXbox.rightTrigger().onTrue(intakeSubsystem.setRollerDutyCycle(0.35));
    driverXbox.leftTrigger().onTrue(intakeSubsystem.setRollerDutyCycle(0));
    driverXbox.x().whileTrue(driveRobotOrientedAnglularVelocity);
    driverXbox.a().onTrue(driveFieldOrientedAnglularVelocity);
    driverXbox.b().onTrue(Commands.runOnce(() -> swerveSubsystem.zeroFieldOrientedHeading(driveAngularVelocity)));
    driverXbox.povLeft().whileTrue(intakeSubsystem.setIntakeAngle());
    driverXbox.povUp().whileTrue(intakeSubsystem.setIntakeZero());

    buttonPad.button(1).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.LeftBack, true));
    buttonPad.button(2).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.LeftFront, true));
    buttonPad.button(3).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.RightBack, true));
    buttonPad.button(4).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.RightFront, true));
    buttonPad.button(5).whileTrue(swerveSubsystem.pathfindToFieldTarget(ClimbTarget.Left, true));
    buttonPad.button(6).whileTrue(swerveSubsystem.pathfindToFieldTarget(ClimbTarget.Right, true));
    buttonPad.button(7).whileTrue(swerveSubsystem.pathfindToFieldTarget(OutpostTarget.Left, true));
    buttonPad.button(8).whileTrue(swerveSubsystem.pathfindToFieldTarget(OutpostTarget.Right, true));

    // driverXbox.a().whileTrue(intakeSubsystem.setState(IntakeState.HOLDING));
    // driverXbox.y().whileTrue(shooterSubsystem.shooterPitchSysId());
    // mechanismXbox.a().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.INTAKING));
    // mechanismXbox.b().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.HOLDING));
    // mechanismXbox.y().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.OUTTAKING));
    // mechanismXbox.x().onTrue(intakeSubsystem.setState(frc.robot.Utils.IntakeState.STOWED));

    // Start / Stop the Phoenix Signal Logger
    // driverXbox.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // driverXbox.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // // SysId test bindings (hold button while test runs)
    // driverXbox.y().whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverXbox.b().whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverXbox.a().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverXbox.x().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
