// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.field;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Utils.FieldTarget;

/** Add your docs here. */
public class HubTarget implements FieldTarget{
    

    public Pose3d getLeftPose() {
        return Constants.LocalizationConstants.leftHubPose;
    }

    public Pose3d getRightPose(){
        return Constants.LocalizationConstants.rightHubPose;
    }

    public List<Integer> getAprilTagIds(){
        return Arrays.asList(Constants.LocalizationConstants.leftHubAprilTagID, Constants.LocalizationConstants.rightHubAprilTagID);
    }

    
    public int getAprilTagId(){
        return 100; //Example april tag id for interface
    }

    public Pose3d getClosestPose(Pose3d robotPose){
        double distanceLeft = robotPose.getTranslation().getDistance(Constants.LocalizationConstants.leftHubPose.getTranslation());
        double distanceRight = robotPose.getTranslation().getDistance(Constants.LocalizationConstants.rightHubPose.getTranslation());
        return distanceLeft < distanceRight ? Constants.LocalizationConstants.leftHubPose : Constants.LocalizationConstants.rightHubPose;
    }

    public Pose3d getTargetPose(){
        return Constants.LocalizationConstants.leftHubPose; //Default for Implementation
    };
}
