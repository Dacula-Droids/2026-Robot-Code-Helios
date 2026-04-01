package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Utils.FieldConstants;
import frc.robot.Utils.FieldConstants.AprilTagIDs;

public enum HubTarget implements FieldTarget {

    Center(AprilTagIDs.getAllianceHubId(),
            FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceHubId()).get()) {
        
        public int getAprilTagId() {
            return ApriltagId;
        }

        
        public Pose3d getTargetPose() {
            return TargetPose;
        }
    };

    public final int ApriltagId;
    public final Pose3d TargetPose;

    private HubTarget(int ApriltagId, Pose3d TargetPose) {
        this.ApriltagId = ApriltagId;
        this.TargetPose = TargetPose;
    }
}