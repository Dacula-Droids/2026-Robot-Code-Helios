// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private static SwerveSubsystem INSTANCE = new SwerveSubsystem();
  public static SwerveSubsystem getInstance(){
    return INSTANCE;
  }

  VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();

  
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;


  public SwerveSubsystem() {
  try {
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.DriveConstants.MAXIMUM_SPEED, new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
  } catch (Exception e)
  {
    throw new RuntimeException();
  }

   SwerveController swerveController = swerveDrive.getSwerveController();
    ZeroGyro();
    setupPathPlanner();
 
  }
  //Low-Level Control, only works when called
  public void driveFieldOriented(ChassisSpeeds velocity){
  swerveDrive.driveFieldOriented(velocity);
  }

  //Works everytime called, 20ms
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void drive(ChassisSpeeds velocity){
    swerveDrive.drive(velocity);
  }

  public Command drive(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.drive(velocity.get());
    });
  }


  public SwerveDrive getSwerveDrive(){
  return swerveDrive;
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }
  
  public void ZeroGyro(){
    swerveDrive.zeroGyro();
  }

  public Rotation2d getOdometryHeading(){
  return swerveDrive.getOdometryHeading();
  } 

  private void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
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
              // drive trains (swerves)
              new PIDConstants(3.35, 0.0, 0),
              // Translation PID constants
              new PIDConstants(2.65, 0.0, 0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            // var alliance = DriverStation.getAlliance();
            // if (alliance.isPresent()) {
            //   return alliance.get() == DriverStation.Alliance.Red;
            // }
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
    //PathfindingCommand.warmupCommand().schedule(); - Deprecated
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  public Command getAutonomousCommand(String pathName){
    return new PathPlannerAuto(pathName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
