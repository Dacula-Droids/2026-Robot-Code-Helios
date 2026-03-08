package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Utils.FieldConstants;
import frc.robot.Utils.FieldConstants.AprilTagIDs;

public enum ClimbTarget implements FieldTarget {

    Left(AprilTagIDs.getAllianceClimbLeftId(),
            FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceClimbLeftId()).get()) {
        @Override
        public int getAprilTagId() {
            return ApriltagId;
        }

        @Override
        public Pose3d getTargetPose() {
            return TargetPose;
        }
    },
    Right(AprilTagIDs.getAllianceClimbRightId(),
            FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceClimbRightId()).get()) {
        @Override
        public int getAprilTagId() {
            return ApriltagId;
        } // Capital T

        @Override
        public Pose3d getTargetPose() {
            return TargetPose;
        }
    };

    public final int ApriltagId;
    public final Pose3d TargetPose;

    private ClimbTarget(int ApriltagId, Pose3d TargetPose) {
        this.ApriltagId = ApriltagId;
        this.TargetPose = TargetPose;
    }
}