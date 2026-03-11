package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Utils.FieldConstants;
import frc.robot.Utils.FieldConstants.AprilTagIDs;

public enum OutpostTarget implements FieldTarget {
    
    Left(AprilTagIDs.getAllianceOutpostLeftId(),
    FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceOutpostLeftId()).get()){
        
        public int getAprilTagId(){ return ApriltagId; } 
        
        public Pose3d getTargetPose(){ return TargetPose; }
    },
    Right(AprilTagIDs.getAllianceOutpostRightId(),
    FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceOutpostRightId()).get()){

        public int getAprilTagId(){ return ApriltagId; } // Capital T
        
        public Pose3d getTargetPose(){ return TargetPose; }
    };
    
    public final int ApriltagId;
    public final Pose3d TargetPose;

    private OutpostTarget(int ApriltagId, Pose3d TargetPose){
        this.ApriltagId = ApriltagId;
        this.TargetPose = TargetPose;
    }
}