// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Utils.ClimbTarget;
import frc.robot.Utils.HubTarget;
import frc.robot.Utils.IntakeState;
import frc.robot.Utils.OutpostTarget;
import frc.robot.Utils.TrenchTarget;
import frc.robot.commands.IntakingPivot;
import frc.robot.commands.MoveAndAimWhileShooting;
import frc.robot.commands.TestPivot;
import frc.robot.commands.ZeroPivot;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterType;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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
  private final VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController mechanismXbox = new CommandXboxController(
      OperatorConstants.kMechanismControllerPort);
  private final CommandGenericHID buttonPad = new CommandGenericHID(
      Constants.OperatorConstants.kButtonBoardControllerPort);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.kSwerveControllerDeadband)
      .scaleTranslation(0.75).scaleRotation(0.35)
      .allianceRelativeControl(false);
  SwerveInputStream driveAngularVelocityNoHeading = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .deadband(OperatorConstants.kSwerveControllerDeadband)
      .scaleTranslation(0.25)
      .allianceRelativeControl(false);
  SwerveInputStream driveRobotOrientedAngularVelocity = driveAngularVelocity.copy().robotRelative(true);

  // public SwerveInputStream getDrivingSwerveInputStream(){
  // return driveAngularVelocity;
  // }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the trigger bindings
    configureBindings();
    configureAutonomous();
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
        getKickerCommand());

    Command goToRightOutpostAndShoot = new SequentialCommandGroup(
        swerveSubsystem.pathfindToFieldTarget(OutpostTarget.Right, true), Commands.waitSeconds(3),
        getKickerCommand());

    Command goToHubAndShoot = new SequentialCommandGroup(
        swerveSubsystem.pathfindToFieldTarget(HubTarget.Center, true),
        startShootingCommand(),
        Commands.waitSeconds(2),
        getKickerCommand());

    NamedCommands.registerCommand("Intake Pivot Down", intakeSubsystem.setIntakeAngle());
    NamedCommands.registerCommand("Intake Pivot Up", intakeSubsystem.setIntakeZero());
    NamedCommands.registerCommand("Run Intake", getIntakingCommand());
    NamedCommands.registerCommand("Stop Intake", stopIntakingCommand());
    NamedCommands.registerCommand("Start Shooting", startShootingCommand());
    NamedCommands.registerCommand("Start Kicker", getKickerCommand());
    NamedCommands.registerCommand("Start Shooter", startShootingCommand());

    // autoChooser.setDefaultOption("GoToLeftOutpostAndShoot",
    // goToLeftOutpostAndShoot);
    // autoChooser.addOption("GoToRightOutpostAndShoot", goToRightOutpostAndShoot);

    autoChooser.addOption("goToHubAndShoot", goToHubAndShoot);
    autoChooser.addOption("RightNeutralZoneSweepThenShoot", rightNeutralZoneSweepThenShoot());

    SmartDashboard.putData("Auto Routine", autoChooser);
  }

  public Command rightNeutralZoneSweepThenShoot() {
    PathPlannerPath neutralZoneSweep;
    try {
      neutralZoneSweep = PathPlannerPath.fromPathFile("RightNeutralZoneBallPickup");
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: NeutralZoneSweep", false);
      return Commands.none();
    }

    return new SequentialCommandGroup(

        swerveSubsystem.pathfindToFieldTarget(HubTarget.Center, true),
        startShootingCommand(),
        Commands.waitSeconds(1.5),
        getKickerCommand(),
        Commands.waitSeconds(3),
        stopShootingCommand(),
        
        swerveSubsystem.driveToPoseThenFollow("RightTrenchPass"),

        new ParallelDeadlineGroup(
            AutoBuilder.followPath(neutralZoneSweep),
            intakeSubsystem.setIntakeAngle(),
            getIntakingCommand()),

        stopIntakingCommand(),
        intakeSubsystem.setIntakeZero(),

        swerveSubsystem.driveToPoseThenFollow("RightTrenchPassBack"),
        startShootingCommand(),
        Commands.waitSeconds(1.5),
        getKickerCommand(),
        Commands.waitSeconds(3.5),
        stopShootingCommand());
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveRobotOrientedAngularVelocity);
    Command defaultIntakeAngle = intakeSubsystem.setIntakeDefault();

    swerveSubsystem.setDefaultCommand(driveRobotOrientedAnglularVelocity);
    // intakeSubsystem.setDefaultCommand(defaultIntakeAngle);

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

    driverXbox.rightBumper().onTrue(getKickerCommand());
    driverXbox.leftBumper().onTrue(stopShootingCommand());

    // // //driverXbox.rightTrigger().onTrue(getIntakingCommand());

    driverXbox.rightTrigger().onTrue(getIntakingCommand());
    // //driverXbox.rightTrigger().onTrue(intakeSubsystem.setIntakePivotDutyCycle(0.037));
    // //ks 0.06, kg 0.037
    driverXbox.leftTrigger().onTrue(stopIntakingCommand());
    driverXbox.povDown().onTrue(intakeSubsystem.setIntakeAngle());
    driverXbox.povUp().onTrue(intakeSubsystem.setIntakeZero());

    // driverXbox.rightTrigger().whileTrue(()->intakeSubsystem.setRollerVelocitySetpoint(RPM.of(2)));

    driverXbox.x().whileTrue(driveRobotOrientedAnglularVelocity);
    driverXbox.a().onTrue(driveFieldOrientedAnglularVelocity);
    driverXbox.b().onTrue(Commands.runOnce(() -> swerveSubsystem.zeroFieldOrientedHeading(driveAngularVelocity)));
    // driverXbox.y().onTrue(startShootingCommand());

    driverXbox.povRight().onTrue(intakeSubsystem.setIntakeAngle());
    driverXbox.povUp().onTrue(intakeSubsystem.setIntakeZero());
    //driverXbox.povDown().onTrue(startShootingCommand());

    buttonPad.button(8).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.LeftBack, true));
    buttonPad.button(6).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.LeftFront, true));
    buttonPad.button(1).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.RightBack, true));
    buttonPad.button(2).whileTrue(swerveSubsystem.pathfindToFieldTarget(TrenchTarget.RightFront, true));
    // //buttonPad.button(5).whileTrue(swerveSubsystem.pathfindToFieldTarget(ClimbTarget.Left,
    // true));
    // //buttonPad.button(3).whileTrue(swerveSubsystem.pathfindToFieldTarget(ClimbTarget.Right,
    // true));
    buttonPad.button(7)
        .onTrue(new MoveAndAimWhileShooting(() -> driverXbox.y().getAsBoolean(), driveAngularVelocityNoHeading));
    // //
    // buttonPad.button(10).whileTrue(swerveSubsystem.pathfindToFieldTarget(OutpostTarget.Right,
    // true));
    // buttonPad.button(4).whileTrue(swerveSubsystem.pathfindToFieldTarget(HubTarget.Center,
    // true));

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

  private Command getKickerCommand() {
    return Commands.sequence(
        kickerSubsystem.setKickerDutyCycle(-1).withTimeout(0.15), kickerSubsystem.setKickerDutyCycle(1));

  }

  private Command stopShootingCommand() {
    return Commands.parallel(
        shooterSubsystem
            .run(() -> shooterSubsystem.setShooterFlywheelVelocitySetpoint(RPM.of(0))),
        indexerSubsystem.setSpinDexerDutyCycle(0), kickerSubsystem.setKickerDutyCycle(0));

  }

  private Command getIntakingCommand() {
    return Commands.parallel(
    intakeSubsystem.run(()-> intakeSubsystem.setRollerVelocitySetpoint(RPM.of(-107))), indexerSubsystem.setSpinDexerDutyCycle(0.8)
    );
  }

  private Command stopIntakingCommand() {
    return Commands.parallel(
        intakeSubsystem.run(() -> intakeSubsystem.setRollerZero()), indexerSubsystem.setSpinDexerDutyCycle(0));
  }

  public Command startShootingCommand() {
    return shooterSubsystem.run(() -> {
      Pose2d currentPose = swerveSubsystem.swerveDrive.getPose();// Pose2d currentPose = new Pose2d(4.6256-1.8542,
                                                                 // 4.0347, new Rotation2d());
      var robotVelocity = swerveSubsystem.swerveDrive.getRobotVelocity();

      var params = shooterSubsystem.getShotParams(currentPose, robotVelocity);

      if (params.isPresent()) {
        // Index one for the speed
        double targetSpeedMPS = params.get()[1];
        shooterSubsystem.setFlywheelVelocityMPS(MetersPerSecond.of(targetSpeedMPS));
      }
    });
  }

  // public Command startShootingCommand(){
  // return shooterSubsystem.run(()-> {
  // shooterSubsystem.setFlywheelVelocityMPS(MetersPerSecond.of(7));
  // // shooterSubsystem.setShooterFlywheelVelocitySetpoint(RPM.of((6000)/2));
  // // shooterSubsystem.setShooterVoltage(0.19);
  // });
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
