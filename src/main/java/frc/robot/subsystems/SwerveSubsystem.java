// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Utils.FieldTarget;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {
  /**
   * The Singleton instance of this SwerveSubsystem. Code should use
   * the {@link #getInstance()} method to get the single instance (rather
   * than trying to construct an instance of this class.)
   */
  private static SwerveSubsystem INSTANCE;
  public final SwerveDrive swerveDrive;
  public final SwerveController swerveController;

  /**
   * Returns the Singleton instance of this SwerveSubsystem. This static method
   * should be used, rather than the constructor, to get the single instance
   * of this class. For example: {@code SwerveSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static SwerveSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SwerveSubsystem();
    }
    return INSTANCE;
  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "KrakenMk4iSwerveConfig"))
          .createSwerveDrive(PhysicalConstants.kMaxSpeed.magnitude());
      swerveController = swerveDrive.getSwerveController();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle.
    swerveDrive.setCosineCompensator(false); // !SwerveDriveTelemetry.isSimulation) Disables cosine compensation for
                                             // simulations since it causes discrepancies not seen in real life.
    // swerveDrive.pushOffsetsToEncoders();
    zeroGyro();
    setupPathPlanner();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints pathConstraints = new PathConstraints(3, 3,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(pose, pathConstraints, 0);
  }

  private void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = false;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(4.25, 0.0, 0),
              // Translation PID constants
              new PIDConstants(4.65, 0.0, 0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command pathFindToFieldTarget(FieldTarget fieldTarget, boolean isOffset) {
    Pose3d tagPose = fieldTarget.getTargetPose();
    Transform3d offset = frc.robot.field.FieldConstants.getFieldTargetOffset(fieldTarget, isOffset);
    Pose2d goalPose = tagPose.plus(offset).toPose2d();
    return driveToPose(goalPose);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void zeroFieldOrientedHeading(SwerveInputStream swerveInputStream) {
    swerveInputStream.translationHeadingOffset(true).translationHeadingOffset(swerveDrive.getOdometryHeading());
  }

  @Override
  public void periodic() {
    LimelightHelpers.PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    // This method will be called once per scheduler run
  }
}