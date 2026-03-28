package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Utils.FieldConstants;
import frc.robot.Utils.FieldConstants.AprilTagIDs;

public enum TrenchTarget implements FieldTarget {

    LeftFront(AprilTagIDs.getAllianceTrenchLeftFrontId(),
            FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceTrenchLeftFrontId()).get()) {
        
        public int getAprilTagId() {
            return ApriltagId;
        }

        
        public Pose3d getTargetPose() {
            return TargetPose;
        }
    },

    RightFront(AprilTagIDs.getAllianceTrenchRightFrontId(),
            FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceTrenchRightFrontId()).get()) {
        @Override
        public int getAprilTagId() {
            return ApriltagId;
        } // Capital T

        @Override
        public Pose3d getTargetPose() {
            return TargetPose;
        }
    },
    LeftBack(AprilTagIDs.getAllianceTrenchLeftBackId(),
            FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceTrenchLeftBackId()).get()) {
        @Override
        public int getAprilTagId() {
            return ApriltagId;
        }

        @Override
        public Pose3d getTargetPose() {
            return TargetPose;
        }
    },
    RightBack(AprilTagIDs.getAllianceTrenchRightBackId(),
            FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceTrenchRightBackId()).get()) {
        @Override
        public int getAprilTagId() {
            return ApriltagId;
        }

        @Override
        public Pose3d getTargetPose() {
            return TargetPose;
        }
    };

    public final int ApriltagId;
    public final Pose3d TargetPose;

    private TrenchTarget(int ApriltagId, Pose3d TargetPose) {
        this.ApriltagId = ApriltagId;
        this.TargetPose = TargetPose;
    }
}