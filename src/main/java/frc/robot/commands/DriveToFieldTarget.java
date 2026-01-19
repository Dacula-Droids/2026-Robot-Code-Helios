// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.FieldTarget;
import frc.robot.field.HubTarget;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToFieldTarget extends Command {
  /** Creates a new DriveToFieldTarget. */
  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  private final FieldTarget target;
  private HubTarget hubTarget;
  private PathfindingCommand pathfindingCommand;
  private Pose2d swervePoseSetpoint;
  
  public DriveToFieldTarget(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, FieldTarget target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.target = target;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = visionSubsystem.hasValidTarget() ? visionSubsystem.getEstimatedPose() : swerveSubsystem.getPose();
    Pose3d currentPose3d = new Pose3d(
      new Translation3d(currentPose.getX(), currentPose.getY(), 0), //0 for height of robot's camera/shooter height, 0 is approximate
      new Rotation3d(0,0, currentPose.getRotation().getRadians())
    );
    
    //Target Pose
    //Pose3d targetPose3d = target.HubTarget() ? hubTarget.getClosestPose(currentPose) : target.getTargetPose();
    
    Pose3d targetPose3d;
    if (target instanceof HubTarget hubTarget){
      targetPose3d = hubTarget.getClosestPose(currentPose3d);
    } else {
      targetPose3d = target.getTargetPose();
    }
    
    //Based on coordinates of target, the function finds the angle from the robot to the target in radians!!!!!!!!
    double endRotation = Math.atan2(
      targetPose3d.getY() - currentPose.getY(),
      targetPose3d.getX() - currentPose.getX()
    );
    
    Pose2d targetPose2d = new Pose2d(
      targetPose3d.getX(),
      targetPose3d.getY(),
      new Rotation3d(0,0,endRotation).toRotation2d() 
    );
    
    //TUNE ALL THE CONSTANTS FOR PATH CONSTRAINTS!!!!!!!
    //Type Casting AutoBuilder.pathfindToPose to PathfindingCommand
    pathfindingCommand = (PathfindingCommand) AutoBuilder.pathfindToPose(targetPose2d, new PathConstraints(swerveSubsystem.getSwerveDrive().getMaximumChassisVelocity(),4.5, swerveSubsystem.getSwerveDrive().getMaximumChassisAngularVelocity(), Units.degreesToRadians(720)), 0);
    
    //Pathfinding Command
    CommandScheduler.getInstance().schedule(pathfindingCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(pathfindingCommand != null){
      pathfindingCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand != null && pathfindingCommand.isFinished();
  }
}
