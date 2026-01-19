// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private static VisionSubsystem INSTANCE = new VisionSubsystem();

  public static VisionSubsystem getInstance(){
    return INSTANCE;
  }

  private static final String LIMELIGHT_NAME = "limelight";


  public VisionSubsystem() {}

  public boolean hasValidTarget(){
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  public Pose2d getEstimatedPose(){
    return LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME); //Blue, as this is where FRC coordinated (0,0) starts, universally accepted across pathplanner and other tools. 
  }
  
  //Subtraction is done for Latency Compensation so everything else is accurate at the current time
  //Compensates for Limelight Latency Delay
  //Used for YAGSL Pose Estimator
  public double getTimestamp(){
    return Timer.getFPGATimestamp()
          - (LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME)/1000)
          - (LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME)/1000);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
